/*
 * This example code looks for the current gamepad state once per frame,
 * and draws a visual representation of it. See 01-joystick-polling for the
 * equivalent example code for the lower-level joystick API.
 *
 * This code is public domain. Feel free to use it for any purpose!
 */

/* Joysticks are low-level interfaces: there's something with a bunch of
   buttons, axes and hats, in no understood order or position. This is
   a flexible interface, but you'll need to build some sort of configuration
   UI to let people tell you what button, etc, does what. On top of this
   interface, SDL offers the "gamepad" API, which works with lots of devices,
   and knows how to map arbitrary buttons and such to look like an
   Xbox/PlayStation/etc gamepad. This is easier, and better, for many games,
   but isn't necessarily a good fit for complex apps and hardware. A flight
   simulator, a realistic racing game, etc, might want the joystick interface
   instead of gamepads. */

/* SDL can handle multiple gamepads, but for simplicity, this program only
   deals with the first gamepad it sees. */

#define SDL_MAIN_USE_CALLBACKS 1  /* use the callbacks instead of main() */
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include "gamepad_front.h"
#include <cstdio>
#include <cstring>
#include <errno.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "lejusdk-lowlevel/leju_sdk.h"
#include "lejusdk-utils/time_utils.hpp"

/* We will use this renderer to draw into this window every frame. */
static SDL_Window *window = NULL;
static SDL_Renderer *renderer = NULL;
static SDL_Texture *texture = NULL;
static SDL_Gamepad *gamepad = NULL;
static SDL_Joystick *joystick = NULL;
static int instance_lock_fd_ = -1;

static bool headless_ = false;
static bool passive_instance_ = false;
static float virtual_gamepad_axis_left_x_ = 0.;
static float virtual_gamepad_axis_left_y_ = 0.;
static float virtual_gamepad_axis_right_x_ = 0.;
static float virtual_gamepad_axis_right_y_ = 0.;
static float virtual_gamepad_axis_left_trigger_ = 0.;
static float virtual_gamepad_axis_right_trigger_ = 0.;

static bool virtual_gamepad_buttons_[25] = {};  // 0-14 standard, 15 M1/misc1, 16-21 paddles/touchpad/M2/misc

// 手柄开机稳定窗口：设备重连/开机的初帧可能携带垃圾轴/键值
static Uint64 gamepad_added_ticks_ = 0;
static bool gamepad_settle_logged_ = false;
static float settle_max_abs_axis_ = 0.f;

#define WINDOW_WIDTH 640
#define WINDOW_HEIGHT 480

namespace {

constexpr const char *kJoystickSingletonName = "lejulab_joystick_singleton";
constexpr Uint64 kGamepadSettleMs = 500;  // 手柄打开后的数据忽略窗口时长

// 清空虚拟手柄数据。手柄断开或被拦截后，兜底分支只清按键、不清摇杆轴，
// 最后一帧残留的非零轴会在无手柄期间持续发布，导致机器人继续移动。
void clearVirtualGamepadInput()
{
    virtual_gamepad_axis_left_x_ = 0.;
    virtual_gamepad_axis_left_y_ = 0.;
    virtual_gamepad_axis_right_x_ = 0.;
    virtual_gamepad_axis_right_y_ = 0.;
    virtual_gamepad_axis_left_trigger_ = 0.;
    virtual_gamepad_axis_right_trigger_ = 0.;
    for (int i = 0; i < 25; i++) {
        virtual_gamepad_buttons_[i] = false;
    }
}

bool acquireInstanceLock()
{
    instance_lock_fd_ = socket(AF_UNIX, SOCK_STREAM, 0);
    if (instance_lock_fd_ < 0) {
        SDL_Log("Failed to create joystick singleton socket: %s", strerror(errno));
        return true;
    }

    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    addr.sun_path[0] = '\0';
    std::snprintf(addr.sun_path + 1, sizeof(addr.sun_path) - 1, "%s", kJoystickSingletonName);

    const socklen_t addr_len =
        static_cast<socklen_t>(offsetof(sockaddr_un, sun_path) + 1 + std::strlen(kJoystickSingletonName));
    if (bind(instance_lock_fd_, reinterpret_cast<const sockaddr *>(&addr), addr_len) == 0) {
        return true;
    }

    if (errno == EADDRINUSE) {
        passive_instance_ = true;
        SDL_Log("Another leju_joystick instance is already running, stay passive.");
        close(instance_lock_fd_);
        instance_lock_fd_ = -1;
        return false;
    }

    SDL_Log("Failed to bind joystick singleton socket: %s", strerror(errno));
    close(instance_lock_fd_);
    instance_lock_fd_ = -1;
    return true;
}

void releaseInstanceLock()
{
    if (instance_lock_fd_ >= 0) {
        close(instance_lock_fd_);
        instance_lock_fd_ = -1;
    }
}

// 北通手柄 NS 模式判定：合法 BFM 模式 16 键，NS 模式 18 键。
// NS 模式轴映射与 Xbox 布局错位，静止时可能上报非零轴，导致站立时机器人莫名移动。
// Xbox 手柄 15/16 键，不受影响。
bool isNsModeGamepad(int num_buttons)
{
    return num_buttons >= 18;
}

}  // namespace

/* This function runs once at startup. */
SDL_AppResult SDL_AppInit(void **appstate, int argc, char *argv[])
{
    char *png_path = NULL;
    SDL_Surface *surface = NULL;
    SDL_IOStream *src = NULL;

    SDL_SetHint(SDL_HINT_MAIN_CALLBACK_RATE, "60");
    SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, "1");

    SDL_SetAppMetadata("Example Input Gamepad Polling", "1.0", "com.example.input-gamepad-polling");

    if (!acquireInstanceLock()) {
        return SDL_APP_CONTINUE;
    }

    if (!SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMEPAD)) {
        SDL_Log("Couldn't initialize SDL: %s", SDL_GetError());
        SDL_Log("Try headless mode.");
        headless_ = true;
        if (!SDL_Init(SDL_INIT_GAMEPAD)) {
            SDL_Log("Couldn't initialize SDL: %s", SDL_GetError());
            return SDL_APP_FAILURE;
        }
    }

    if (headless_) {
        goto Init_lejusdk;
    }
    if (!SDL_CreateWindowAndRenderer("examples/input/gamepad-polling", WINDOW_WIDTH, WINDOW_HEIGHT, SDL_WINDOW_RESIZABLE, &window, &renderer)) {
        SDL_Log("Couldn't create window/renderer: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }

    if (!SDL_SetRenderLogicalPresentation(renderer, WINDOW_WIDTH, WINDOW_HEIGHT, SDL_LOGICAL_PRESENTATION_STRETCH)) {
        return SDL_APP_FAILURE;
    }

    /* Textures are pixel data that we upload to the video hardware for fast drawing. Lots of 2D
       engines refer to these as "sprites." We'll do a static texture (upload once, draw many
       times) with data from a bitmap file. */

    /* SDL_Surface is pixel data the CPU can access. SDL_Texture is pixel data the GPU can access.
       Load a .png into a surface, move it to a texture from there. */

    src = SDL_IOFromConstMem(gamepad_front_png, gamepad_front_png_len);
    if (!src) {
        SDL_Log("Couldn't set io: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }

    surface = SDL_LoadPNG_IO(src, true);
    if (!surface) {
        SDL_Log("Couldn't load bitmap: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }

    texture = SDL_CreateTextureFromSurface(renderer, surface);
    if (!texture) {
        SDL_Log("Couldn't create static texture: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }

    SDL_DestroySurface(surface);  /* done with this, the texture has a copy of the pixels now. */

Init_lejusdk:
    leju::RobotVersion robot_version = leju::RobotVersion::from_env();
    if (!leju::GlobalRobot::init_env(robot_version)) {
        SDL_Log("Call leju::GlobalRobot::init_env() failed.");
        return SDL_APP_FAILURE; // TODO deal with this result
    }
    
    return SDL_APP_CONTINUE;  /* carry on with the program! */
}

/* This function runs when a new event (mouse input, keypresses, etc) occurs. */
SDL_AppResult SDL_AppEvent(void *appstate, SDL_Event *event)
{
    if (passive_instance_) {
        if (event->type == SDL_EVENT_QUIT) {
            return SDL_APP_SUCCESS;
        }
        return SDL_APP_CONTINUE;
    }

    if (event->type == SDL_EVENT_QUIT) {
        return SDL_APP_SUCCESS;  /* end the program, reporting success to the OS. */
    } else if (event->type == SDL_EVENT_GAMEPAD_ADDED && gamepad == NULL) {
        gamepad = SDL_OpenGamepad(event->gdevice.which);
        if (!gamepad) {
            SDL_Log("Failed to open gamepad ID %u: %s", (unsigned int) event->gdevice.which, SDL_GetError());
            return SDL_APP_CONTINUE;
        }
        SDL_Log("Success to open gamepad ID %u", (unsigned int) event->gdevice.which);
        joystick = SDL_GetGamepadJoystick(gamepad);
        const char *name = SDL_GetGamepadName(gamepad);
        const int num_buttons = joystick ? SDL_GetNumJoystickButtons(joystick) : -1;
        if (isNsModeGamepad(num_buttons)) {
            // NS 模式手柄：关闭设备，pub 前禁止其数据发出
            SDL_Log("Block NS mode gamepad '%s' (%d buttons), publish disabled", name, num_buttons);
            SDL_CloseGamepad(gamepad);
            gamepad = NULL;
            joystick = NULL;
            clearVirtualGamepadInput();
            return SDL_APP_CONTINUE;
        }
        // 记录打开时间与设备信息，进入开机稳定窗口
        gamepad_added_ticks_ = SDL_GetTicks();
        gamepad_settle_logged_ = false;
        settle_max_abs_axis_ = 0.f;
        SDL_Log("Open gamepad '%s' (%d buttons)", name, num_buttons);
    } else if (event->type == SDL_EVENT_GAMEPAD_REMOVED) {
        if (gamepad && (SDL_GetGamepadID(gamepad) == event->gdevice.which)) {
            joystick = NULL;
            SDL_CloseGamepad(gamepad);
            gamepad = NULL;
            // 断开后清空残留轴值，避免关机窗口持续发布非零轴
            clearVirtualGamepadInput();
        }
    }
    return SDL_APP_CONTINUE;  /* carry on with the program! */
}

/* This function runs once per frame, and is the heart of the program. */
SDL_AppResult SDL_AppIterate(void *appstate)
{
    if (passive_instance_) {
        usleep(200000);
        return SDL_APP_CONTINUE;
    }

    const char *text = "Plug in a gamepad, please.";
    static Uint64 leftthumblast = 0xFFFFFFFF;
    static Uint64 rightthumblast = 0xFFFFFFFF;
    const Uint64 now = SDL_GetTicks();
    Sint16 axis_x, axis_y;
    float x, y;
    int i;

    // SDL_GetMouseState(&x, &y);
    // std::cout << "Mouse pos: " << x << ", " << y << "." << std::endl;
    // TODO lejusdk use gamepad instead of joystick
    leju::JoyData joy;

    if (gamepad) {
        /* note that you can get input as events, instead of polling, which is
        better since it won't miss button presses if the system is lagging,
        but often times checking the current state per-frame is good enough,
        and maybe better if you'd rather _drop_ inputs due to lag. */
        if ((now - gamepad_added_ticks_) < kGamepadSettleMs) {
            // 开机稳定窗口：忽略设备初帧数据，只发布零轴，
            // 并记录窗口内最大轴幅值用于真机诊断
            for (i = 0; i < 6; i++) {
                const float v =
                    SDL_GetGamepadAxis(gamepad, (SDL_GamepadAxis) i) / 32767.0f;
                const float abs_v = v < 0.f ? -v : v;
                if (abs_v > settle_max_abs_axis_) {
                    settle_max_abs_axis_ = abs_v;
                }
            }
            clearVirtualGamepadInput();
        } else {
            if (!gamepad_settle_logged_) {
                gamepad_settle_logged_ = true;
                SDL_Log("Gamepad settle window done, max abs axis during settle = %.3f",
                        settle_max_abs_axis_);
            }
            for (i = 0; i < 25; i++) {
                virtual_gamepad_buttons_[i] = SDL_GetGamepadButton(gamepad, (SDL_GamepadButton) i);
            }
            // M1(raw13) M2(raw14): SDL 映射为 L3(7) R3(8), 重定向到 misc1/misc2
            if (joystick) {
                bool m1_pressed = SDL_GetJoystickButton(joystick, 13);
                bool m2_pressed = SDL_GetJoystickButton(joystick, 14);
                if (m1_pressed) {
                    virtual_gamepad_buttons_[7] = false;
                    virtual_gamepad_buttons_[15] = true;
                }
                if (m2_pressed) {
                    virtual_gamepad_buttons_[8] = false;
                    virtual_gamepad_buttons_[21] = true;
                }
            }
            virtual_gamepad_axis_left_x_ = (SDL_GetGamepadAxis(gamepad, SDL_GAMEPAD_AXIS_LEFTX) / 32767.0f);
            virtual_gamepad_axis_left_y_ = (SDL_GetGamepadAxis(gamepad, SDL_GAMEPAD_AXIS_LEFTY) / 32767.0f);
            virtual_gamepad_axis_right_x_ = (SDL_GetGamepadAxis(gamepad, SDL_GAMEPAD_AXIS_RIGHTX) / 32767.0f);
            virtual_gamepad_axis_right_y_ = (SDL_GetGamepadAxis(gamepad, SDL_GAMEPAD_AXIS_RIGHTY) / 32767.0f);
            virtual_gamepad_axis_left_trigger_ = (SDL_GetGamepadAxis(gamepad, SDL_GAMEPAD_AXIS_LEFT_TRIGGER) / 32767.0f);
            virtual_gamepad_axis_right_trigger_ = (SDL_GetGamepadAxis(gamepad, SDL_GAMEPAD_AXIS_RIGHT_TRIGGER) / 32767.0f);
        }
    } else {
        int numkeys;
        const bool *key_states = SDL_GetKeyboardState(&numkeys);
        SDL_MouseButtonFlags flags = SDL_GetMouseState(&x, &y);

        if (key_states[SDL_SCANCODE_W]) {
            virtual_gamepad_axis_left_y_ -= 1./30;
        }
        if (key_states[SDL_SCANCODE_S]) {
            virtual_gamepad_axis_left_y_ += 1./30;
        }
        if (key_states[SDL_SCANCODE_A]) {
            virtual_gamepad_axis_left_x_ -= 1./30;
        }
        if (key_states[SDL_SCANCODE_D]) {
            virtual_gamepad_axis_left_x_ += 1./30;
        }
        if (key_states[SDL_SCANCODE_I]) {
            virtual_gamepad_axis_right_y_ -= 1./30;
        }
        if (key_states[SDL_SCANCODE_K]) {
            virtual_gamepad_axis_right_y_ += 1./30;
        }
        if (key_states[SDL_SCANCODE_J] || key_states[SDL_SCANCODE_Q]) {
            virtual_gamepad_axis_right_x_ -= 1./30;
        }
        if (key_states[SDL_SCANCODE_L] || key_states[SDL_SCANCODE_E]) {
            virtual_gamepad_axis_right_x_ += 1./30;
        }
        if (key_states[SDL_SCANCODE_SPACE]) {
            clearVirtualGamepadInput();
        }

        if (SDL_BUTTON_LMASK & flags) {
            if (x>=107-30+15 && x<107+30+15 && y>=252-30+15 && y<252+30+15) {
                virtual_gamepad_axis_left_x_ = (x-107-15) / 30.;
                virtual_gamepad_axis_left_y_ = (y-252-15) / 30.;
            }
            if (x>=397-30+15 && x<397+30+15 && y>=370-30+15 && y<370+30+15) {
                virtual_gamepad_axis_right_x_ = (x-397-15) / 30.;
                virtual_gamepad_axis_right_y_ = (y-370-15) / 30.;
            }
        }

        virtual_gamepad_axis_left_x_ = std::max(std::min(virtual_gamepad_axis_left_x_, float(1.)), float(-1.));
        virtual_gamepad_axis_left_y_ = std::max(std::min(virtual_gamepad_axis_left_y_, float(1.)), float(-1.));
        virtual_gamepad_axis_right_x_ = std::max(std::min(virtual_gamepad_axis_right_x_, float(1.)), float(-1.));
        virtual_gamepad_axis_right_y_ = std::max(std::min(virtual_gamepad_axis_right_y_, float(1.)), float(-1.));
        virtual_gamepad_axis_left_trigger_ = std::max(std::min(virtual_gamepad_axis_left_trigger_, float(1.)), float(-1.));
        virtual_gamepad_axis_right_trigger_ = std::max(std::min(virtual_gamepad_axis_right_trigger_, float(1.)), float(-1.));

        for (int i=0; i<25; i++) {
            virtual_gamepad_buttons_[i] = false;
        }
        if (key_states[SDL_SCANCODE_P]) virtual_gamepad_buttons_[9] = true;   // P -> LB
        if (key_states[SDL_SCANCODE_B]) virtual_gamepad_buttons_[4] = true;   // B -> BACK
        if (key_states[SDL_SCANCODE_O] || key_states[SDL_SCANCODE_F]) virtual_gamepad_buttons_[6] = true;  // O/F -> START
        if (key_states[SDL_SCANCODE_C]) virtual_gamepad_buttons_[0] = true;   // C -> A (south)
        if (key_states[SDL_SCANCODE_T]) virtual_gamepad_buttons_[1] = true;   // T -> B (east)
        if (key_states[SDL_SCANCODE_R]) virtual_gamepad_buttons_[3] = true;   // R -> Y (north)
        if (key_states[SDL_SCANCODE_G]) virtual_gamepad_buttons_[5] = true;   // G -> GUIDE
        // 补全虚拟手柄按键映射, 便于键盘测全套触发层
        if (key_states[SDL_SCANCODE_V]) virtual_gamepad_buttons_[2]  = true;  // V -> X (west)
        if (key_states[SDL_SCANCODE_Y]) virtual_gamepad_buttons_[10] = true;  // Y -> RB (right_shoulder)
        if (key_states[SDL_SCANCODE_UP])    virtual_gamepad_buttons_[11] = true;  // ↑ -> DPAD_UP
        if (key_states[SDL_SCANCODE_DOWN])  virtual_gamepad_buttons_[12] = true;  // ↓ -> DPAD_DOWN
        if (key_states[SDL_SCANCODE_LEFT])  virtual_gamepad_buttons_[13] = true;  // ← -> DPAD_LEFT
        if (key_states[SDL_SCANCODE_RIGHT]) virtual_gamepad_buttons_[14] = true;  // → -> DPAD_RIGHT
        if (key_states[SDL_SCANCODE_Z]) virtual_gamepad_buttons_[15] = true;        // Z -> M1 (misc1)
        if (key_states[SDL_SCANCODE_H]) virtual_gamepad_buttons_[21] = true;        // H -> M2 (misc2)
        // 扳机(轴): N=LT, M=RT, 按住=1.0(超过 trigger_threshold 0.5 即视为按下)
        virtual_gamepad_axis_left_trigger_  = key_states[SDL_SCANCODE_N] ? 1.0f : 0.0f;
        virtual_gamepad_axis_right_trigger_ = key_states[SDL_SCANCODE_M] ? 1.0f : 0.0f;

        if (SDL_BUTTON_LMASK & flags) {
            if (x>= 377 && x < 377+32 && y>=228 && y<228+29) {
                virtual_gamepad_buttons_[6] = true;
            }
            if (x>=235 && x<235+32 && y>=228 && y<228+29) {
                virtual_gamepad_buttons_[4] = true;
            }
            if (x>=285 && x<285+69 && y>=195 && y<195+69) {
                virtual_gamepad_buttons_[5] = true;
            }
        }
    }
    auto& robot = leju::GlobalRobot::getInstance();
    // 填充轴数据
    joy.axes.left_x = virtual_gamepad_axis_left_x_;
    joy.axes.left_y = virtual_gamepad_axis_left_y_;
    joy.axes.right_x = virtual_gamepad_axis_right_x_;
    joy.axes.right_y = virtual_gamepad_axis_right_y_;
    joy.axes.left_trigger = virtual_gamepad_axis_left_trigger_;
    joy.axes.right_trigger = virtual_gamepad_axis_right_trigger_;
    // 填充按键数据
    joy.buttons.south = static_cast<int32_t>(virtual_gamepad_buttons_[0]);
    joy.buttons.east = static_cast<int32_t>(virtual_gamepad_buttons_[1]);
    joy.buttons.west = static_cast<int32_t>(virtual_gamepad_buttons_[2]);
    joy.buttons.north = static_cast<int32_t>(virtual_gamepad_buttons_[3]);
    joy.buttons.back = static_cast<int32_t>(virtual_gamepad_buttons_[4]);
    joy.buttons.guide = static_cast<int32_t>(virtual_gamepad_buttons_[5]);
    joy.buttons.start = static_cast<int32_t>(virtual_gamepad_buttons_[6]);
    joy.buttons.left_stick = static_cast<int32_t>(virtual_gamepad_buttons_[7]);
    joy.buttons.right_stick = static_cast<int32_t>(virtual_gamepad_buttons_[8]);
    joy.buttons.left_shoulder = static_cast<int32_t>(virtual_gamepad_buttons_[9]);
    joy.buttons.right_shoulder = static_cast<int32_t>(virtual_gamepad_buttons_[10]);
    joy.buttons.dpad_up = static_cast<int32_t>(virtual_gamepad_buttons_[11]);
    joy.buttons.dpad_down = static_cast<int32_t>(virtual_gamepad_buttons_[12]);
    joy.buttons.dpad_left = static_cast<int32_t>(virtual_gamepad_buttons_[13]);
    joy.buttons.dpad_right = static_cast<int32_t>(virtual_gamepad_buttons_[14]);
    joy.buttons.misc1 = static_cast<int32_t>(virtual_gamepad_buttons_[15]);
    joy.buttons.misc2 = static_cast<int32_t>(virtual_gamepad_buttons_[21]);  // SDL index 21 = MISC2/M2
    // 设置时间戳
    joy.timestamp = leju::common::GetUnixTimestampS();
    robot.publishJoyData(joy);


    if (gamepad) {  /* we have a stick opened? */
        text = SDL_GetGamepadName(gamepad);
    } else {
        text = "no gamepad, use keyboard / mouse input instead";
    }

    /* where to draw the buttons */
    const SDL_FRect buttons[] = {
        { 497, 266, 38,  38 },   /* SDL_GAMEPAD_BUTTON_SOUTH */
        { 550, 217, 38,  38 },   /* SDL_GAMEPAD_BUTTON_EAST */
        { 445, 221, 38,  38 },   /* SDL_GAMEPAD_BUTTON_WEST */
        { 499, 173, 38,  38 },   /* SDL_GAMEPAD_BUTTON_NORTH */
        { 235, 228, 32,  29 },   /* SDL_GAMEPAD_BUTTON_BACK */
        { 287, 195, 69,  69 },   /* SDL_GAMEPAD_BUTTON_GUIDE */
        { 377, 228, 32,  29 },   /* SDL_GAMEPAD_BUTTON_START */
        { 91,  234, 63,  63 },   /* SDL_GAMEPAD_BUTTON_LEFT_STICK */
        { 381, 354, 63,  63 },   /* SDL_GAMEPAD_BUTTON_RIGHT_STICK */
        { 74,  73,  102, 29 },   /* SDL_GAMEPAD_BUTTON_LEFT_SHOULDER */
        { 468, 73,  102, 29 },   /* SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER */
        { 207, 316, 32,  32 },   /* SDL_GAMEPAD_BUTTON_DPAD_UP */
        { 207, 384, 32,  32 },   /* SDL_GAMEPAD_BUTTON_DPAD_DOWN */
        { 173, 351, 32,  32 },   /* SDL_GAMEPAD_BUTTON_DPAD_LEFT */
        { 242, 351, 32,  32 },   /* SDL_GAMEPAD_BUTTON_DPAD_RIGHT */
        { 310, 286, 23,  27 },   /* SDL_GAMEPAD_BUTTON_MISC1 */
        /* there are other buttons: paddles on the back of the gamepad, touchpads, etc, but this is good enough for now. */
    };

    if (headless_) {
        goto Exit; // TODO: add more console print on headless mode
    }
    SDL_SetRenderDrawColor(renderer, 0xFF, 0xFF, 0xFF, 0xFF);  /* white */
    SDL_RenderClear(renderer);

    

    SDL_RenderTexture(renderer, texture, NULL, NULL);  /* draw the gamepad picture to the whole window. */
    /* draw green boxes over buttons that are currently pressed. */
    SDL_SetRenderDrawColor(renderer, 0x00, 0xFF, 0x00, 0xFF);  /* green */
    for (i = 0; i < SDL_arraysize(buttons); i++) {
        if (virtual_gamepad_buttons_[i]) {
            SDL_RenderFillRect(renderer, &buttons[i]);
        }
    }

    SDL_SetRenderDrawColor(renderer, 0xFF, 0xFF, 0x00, 0xFF);  /* yellow */
    /* left thumb axis. */
    axis_x = virtual_gamepad_axis_left_x_ * 32767;
    axis_y = virtual_gamepad_axis_left_y_ * 32767;
    {
        const SDL_FRect box = { 107 + ((axis_x / 32767.0f) * 30.0f), 252 + ((axis_y / 32767.0f) * 30.0f), 30, 30 };
        SDL_RenderFillRect(renderer, &box);
    }

    /* right thumb axis. */
    axis_x = virtual_gamepad_axis_right_x_ * 32767;
    axis_y = virtual_gamepad_axis_right_y_ * 32767;
    {
        const SDL_FRect box = { 397 + ((axis_x / 32767.0f) * 30.0f), 370 + ((axis_y / 32767.0f) * 30.0f), 30, 30 };
        SDL_RenderFillRect(renderer, &box);
    }

    /* left trigger. */
    axis_y = virtual_gamepad_axis_left_trigger_ * 32767;
    if (axis_y > 1000) {   /* zero means unpressed, but it might be a little off zero... */
        const float height = ((axis_y / 32767.0f) * 65.0f);
        const SDL_FRect box = { 127, 1 + (65.0f - height), 37, height };
        SDL_RenderFillRect(renderer, &box);
    }

    /* right trigger. */
    axis_y = virtual_gamepad_axis_right_trigger_ * 32767;
    if (axis_y > 1000) {   /* zero means unpressed, but it might be a little off zero... */
        const float height = ((axis_y / 32767.0f) * 65.0f);
        const SDL_FRect box = { 481, 1 + (65.0f - height), 37, height };
        SDL_RenderFillRect(renderer, &box);
    }

    x = (((float) WINDOW_WIDTH) - (SDL_strlen(text) * SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE)) / 2.0f;
    y = (float) (WINDOW_HEIGHT - (SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE + 2));
    SDL_SetRenderDrawColor(renderer, 0x00, 0x00, 0xFF, 0xFF);  /* blue */
    SDL_RenderDebugText(renderer, x, y, text);

    if (!gamepad) {
        SDL_SetRenderDrawColor(renderer, 0x00, 0x00, 0xFF, 0xFF);
        SDL_RenderDebugText(renderer, 107+15 - (1 * SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE)/2., 252+15-20-SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE/2., "W");
        SDL_RenderDebugText(renderer, 107+15-20 - (1 * SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE)/2., 252+15-SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE/2., "A");
        SDL_RenderDebugText(renderer, 107+15 - (1 * SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE)/2., 252+15+20-SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE/2., "S");
        SDL_RenderDebugText(renderer, 107+15+20 - (1 * SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE)/2., 252+15-SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE/2., "D");
        SDL_RenderDebugText(renderer, 397+15 - (1 * SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE)/2., 370+15-20-SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE/2., "I");
        SDL_RenderDebugText(renderer, 397+15-20 - (3 * SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE)/2., 370+15-SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE/2., "J/Q");
        SDL_RenderDebugText(renderer, 397+15 - (1 * SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE)/2., 370+15+20-SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE/2., "K");
        SDL_RenderDebugText(renderer, 397+15+20 - (3 * SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE)/2., 370+15-SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE/2., "L/E");
        SDL_RenderDebugText(renderer, 74+51 - (1 * SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE)/2., 73+14-SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE/2., "P");
        SDL_RenderDebugText(renderer, 235+16 - (1 * SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE)/2., 228+14-SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE/2., "B");
        SDL_RenderDebugText(renderer, 377+16 - (3 * SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE)/2., 228+14-SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE/2., "O/F");
        SDL_RenderDebugText(renderer, 497+19 - (1 * SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE)/2., 266+19-SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE/2., "C");
        SDL_RenderDebugText(renderer, 499+19 - (1 * SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE)/2., 173+19-SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE/2., "R");
        SDL_RenderDebugText(renderer, 550+19 - (1 * SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE)/2., 217+19-SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE/2., "T");
        SDL_RenderDebugText(renderer, 287+34 - (1 * SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE)/2., 195+34-SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE/2., "G");
        text = "press space to reset axes";
        x = (((float) WINDOW_WIDTH) - (SDL_strlen(text) * SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE)) / 2.0f;
        y = (float) (WINDOW_HEIGHT - (3*SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE + 2));
        SDL_RenderDebugText(renderer, x, y, text);
    }
    SDL_RenderPresent(renderer);
Exit:
    return SDL_APP_CONTINUE;  /* carry on with the program! */
}

/* This function runs once at shutdown. */
void SDL_AppQuit(void *appstate, SDL_AppResult result)
{
    releaseInstanceLock();
    SDL_DestroyTexture(texture);
    SDL_CloseGamepad(gamepad);
    /* SDL will clean up the window/renderer for us. */
}
