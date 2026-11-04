import numpy as np


class SupportFootDriftCompensator:
    """
    Contact-aware X-direction common-mode drift compensation V2.

    V2 核心原则
    ------------------------------------------------------------
    1. 使用脚部高度 + XY/Z速度检测接触状态。
    2. 每只脚维护一个 persistent world anchor。
    3. 同一只脚重新落地时：
         - 与旧 anchor 距离 <= step_update_thresh：
           认为是原地踏步，不更新 anchor。
         - 距离 > step_update_thresh：
           认为是真实迈步，更新 anchor。
    4. 同一时刻只使用一个 active support foot 计算漂移。
    5. 双脚同时接触时，不再左右脚 correction 求平均，
       而是保持之前的 active support。
    6. 当前 V2 只修正 X，不修正 Y。
    7. 对整个人体 skeleton 统一增加同一个 X correction，
       因此不改变人体各 segment 之间的相对几何关系。

    目标
    ------------------------------------------------------------
    原地踏步：
        抑制 PICO global X drift，避免机器人不断向前走。

    正常行走：
        新落脚位置超过 step gate 后更新 anchor，
        尽量保留真实 forward locomotion。
    """

    def __init__(
        self,
        fps=100.0,

        # ------------------------------------------------------
        # Contact hysteresis
        # ------------------------------------------------------
        height_enter=0.040,
        height_keep=0.070,

        vz_enter=0.45,
        vz_keep=0.80,

        vxy_enter=0.45,
        vxy_keep=0.80,

        enter_frames=3,
        exit_frames=3,

        # ------------------------------------------------------
        # Drift correction
        # ------------------------------------------------------
        correction_tau=0.15,
        max_correction_rate=0.12,
        max_correction=0.08,

        # ------------------------------------------------------
        # Step gate
        #
        # 同一只脚新的 corrected landing point
        # 相对旧 anchor 超过该距离，
        # 才认为是真正迈了一步。
        # ------------------------------------------------------
        step_update_thresh=0.08,
    ):
        self.default_dt = 1.0 / float(fps)

        # Contact
        self.height_enter = float(height_enter)
        self.height_keep = float(height_keep)

        self.vz_enter = float(vz_enter)
        self.vz_keep = float(vz_keep)

        self.vxy_enter = float(vxy_enter)
        self.vxy_keep = float(vxy_keep)

        self.enter_frames = int(enter_frames)
        self.exit_frames = int(exit_frames)

        # Correction
        self.correction_tau = float(correction_tau)
        self.max_correction_rate = float(max_correction_rate)
        self.max_correction = float(max_correction)

        # Step gate
        self.step_update_thresh = float(step_update_thresh)

        self.reset()

    # ==========================================================
    # Reset
    # ==========================================================
    def reset(self):
        """
        Calibration / restart 后必须 reset。

        清空：
            contact history
            foot velocity history
            foot anchors
            active support
            correction
            ground estimate
        """

        self.prev_time = None

        self.prev_point = {
            "Left": None,
            "Right": None,
        }

        self.contact = {
            "Left": False,
            "Right": False,
        }

        self.enter_count = {
            "Left": 0,
            "Right": 0,
        }

        self.exit_count = {
            "Left": 0,
            "Right": 0,
        }

        # ------------------------------------------------------
        # Persistent world anchors
        #
        # anchor = raw foot position + current correction
        # ------------------------------------------------------
        self.anchor_xy = {
            "Left": None,
            "Right": None,
        }

        # ------------------------------------------------------
        # Current active support foot:
        #
        # None
        # "Left"
        # "Right"
        # ------------------------------------------------------
        self.active_support = None

        # ------------------------------------------------------
        # V2 correction:
        #
        # 仍保留二维数组方便接口兼容，
        # 但 correction_xy[1] 永远设为 0。
        # ------------------------------------------------------
        self.correction_xy = np.zeros(
            2,
            dtype=np.float64,
        )

        self.ground_z = None

    # ==========================================================
    # Foot representative point
    # ==========================================================
    @staticmethod
    def _get_foot_point(frame, side):
        """
        使用 Foot + ToeBase 构造脚部代表点。

        Parameters
        ----------
        frame:
            GMR FrameDict

        side:
            "Left" / "Right"

        Returns
        -------
        point:
            3D representative foot point

        contact_z:
            用于接触检测的最低脚部高度
        """

        foot_name = f"{side}Foot"
        toe_name = f"{side}ToeBase"

        foot = np.asarray(
            frame[foot_name][0],
            dtype=np.float64,
        )

        if toe_name in frame:

            toe = np.asarray(
                frame[toe_name][0],
                dtype=np.float64,
            )

            # --------------------------------------------------
            # XY / velocity 用 Foot + ToeBase 平均，
            # 降低单点 tracking 抖动。
            # --------------------------------------------------
            point = 0.5 * (
                foot + toe
            )

            # --------------------------------------------------
            # Contact height 使用最低点。
            # --------------------------------------------------
            contact_z = min(
                float(foot[2]),
                float(toe[2]),
            )

        else:

            point = foot.copy()

            contact_z = float(
                foot[2]
            )

        return point, contact_z

    # ==========================================================
    # Contact hysteresis
    # ==========================================================
    def _update_contact(
        self,
        side,
        point,
        contact_z,
        vel,
    ):
        """
        Contact hysteresis + debounce.

        未接触：
            使用 enter thresholds。

        已接触：
            使用更宽松的 keep thresholds。

        避免：
            1/0/1/0 contact chatter。
        """

        state = self.contact[side]

        z_rel = (
            contact_z
            - self.ground_z
        )

        vxy = float(
            np.linalg.norm(
                vel[:2]
            )
        )

        vz = abs(
            float(vel[2])
        )

        # ======================================================
        # Currently NOT contact
        # ======================================================
        if not state:

            candidate = (
                z_rel < self.height_enter
                and vxy < self.vxy_enter
                and vz < self.vz_enter
            )

            if candidate:

                self.enter_count[side] += 1

            else:

                self.enter_count[side] = 0

            entered = False

            if (
                self.enter_count[side]
                >= self.enter_frames
            ):

                state = True
                entered = True

                self.enter_count[side] = 0

            exited = False

        # ======================================================
        # Currently contact
        # ======================================================
        else:

            candidate = (
                z_rel < self.height_keep
                and vxy < self.vxy_keep
                and vz < self.vz_keep
            )

            if candidate:

                self.exit_count[side] = 0

            else:

                self.exit_count[side] += 1

            entered = False
            exited = False

            if (
                self.exit_count[side]
                >= self.exit_frames
            ):

                state = False
                exited = True

                self.exit_count[side] = 0

        self.contact[side] = state

        return (
            state,
            entered,
            exited,
            z_rel,
            vxy,
            vz,
        )

    # ==========================================================
    # Anchor update
    # ==========================================================
    def _update_anchor_on_enter(
        self,
        side,
        point,
    ):
        """
        某只脚重新进入 contact 时判断：
            原地踏步？
            还是真实迈步？

        candidate anchor 使用：

            raw_foot_xy + current_correction_xy

        这样支撑切换时 global corrected trajectory 连续。

        Returns
        -------
        updated:
            是否更新 anchor

        step_dist:
            candidate 与旧 anchor 的 XY 距离

        reason:
            "init"
            "in_place"
            "real_step"
        """

        candidate = (
            point[:2]
            + self.correction_xy
        )

        # ======================================================
        # First anchor
        # ======================================================
        if self.anchor_xy[side] is None:

            self.anchor_xy[side] = (
                candidate.copy()
            )

            return (
                True,
                0.0,
                "init",
            )

        # ======================================================
        # Compare with persistent anchor
        # ======================================================
        step_dx = float(
            abs(
                candidate[0]
                - self.anchor_xy[side][0]
            )
        )

        if (
            step_dx
            > self.step_update_thresh
        ):

            self.anchor_xy[side][0] = (
                candidate[0]
            )

            return (
                True,
                step_dist,
                "real_step",
            )

        # ======================================================
        # In-place step
        #
        # 保持旧 anchor。
        # ======================================================
        return (
            False,
            step_dist,
            "in_place",
        )

    # ==========================================================
    # Active support selection
    # ==========================================================
    def _select_active_support(
        self,
        lc,
        rc,
        l_enter,
        r_enter,
        l_vxy,
        r_vxy,
        l_vz,
        r_vz,
    ):
        """
        支撑脚状态机。

        典型切换：

            L1 R0
            active=Left

               ↓ Right landing

            L1 R1
            active仍然=Left

               ↓ Left lift

            L0 R1
            active=Right

        双支撑期间不进行 Left/Right correction 平均。
        """

        previous_support = (
            self.active_support
        )

        # ======================================================
        # No active support
        # ======================================================
        if self.active_support is None:

            # Left only
            if lc and not rc:

                self.active_support = (
                    "Left"
                )

            # Right only
            elif rc and not lc:

                self.active_support = (
                    "Right"
                )

            # Double support
            elif lc and rc:

                # ----------------------------------------------
                # 如果只有某一只脚刚进入 contact，
                # 优先使用这只脚。
                # ----------------------------------------------
                if (
                    l_enter
                    and not r_enter
                ):

                    self.active_support = (
                        "Left"
                    )

                elif (
                    r_enter
                    and not l_enter
                ):

                    self.active_support = (
                        "Right"
                    )

                else:

                    # ------------------------------------------
                    # 初始站立时通常 L/R 同时 contact。
                    #
                    # 选择运动速度更小的一只脚。
                    # ------------------------------------------
                    left_score = (
                        l_vxy
                        + abs(l_vz)
                    )

                    right_score = (
                        r_vxy
                        + abs(r_vz)
                    )

                    if (
                        left_score
                        <= right_score
                    ):

                        self.active_support = (
                            "Left"
                        )

                    else:

                        self.active_support = (
                            "Right"
                        )

        # ======================================================
        # Left currently active
        # ======================================================
        elif (
            self.active_support
            == "Left"
        ):

            # --------------------------------------------------
            # Left still in contact:
            #
            # L1R0
            # L1R1
            #
            # 均保持 Left。
            # --------------------------------------------------
            if lc:

                pass

            # --------------------------------------------------
            # Left lifted, Right supporting
            # --------------------------------------------------
            elif rc:

                self.active_support = (
                    "Right"
                )

            # --------------------------------------------------
            # Flight phase
            # --------------------------------------------------
            else:

                self.active_support = None

        # ======================================================
        # Right currently active
        # ======================================================
        elif (
            self.active_support
            == "Right"
        ):

            # Right still supporting
            if rc:

                pass

            # Right lifted, Left supporting
            elif lc:

                self.active_support = (
                    "Left"
                )

            # Flight
            else:

                self.active_support = None

        switched = (
            previous_support
            != self.active_support
        )

        return (
            previous_support,
            self.active_support,
            switched,
        )

    # ==========================================================
    # Main
    # ==========================================================
    def apply(
        self,
        frame,
        timestamp=None,
    ):
        """
        Parameters
        ----------
        frame:
            GMR FrameDict

        timestamp:
            capture timestamp in seconds

        Returns
        -------
        corrected_frame:
            输入 GMR skeleton + uniform X correction

        debug_info:
            contact / anchor / active support /
            correction / step detection 等调试数据
        """

        # ======================================================
        # Deep copy
        # ======================================================
        out = {
            name: [
                np.asarray(
                    value[0],
                    dtype=np.float64,
                ).copy(),
                np.asarray(
                    value[1],
                    dtype=np.float64,
                ).copy(),
            ]
            for name, value
            in frame.items()
        }

        # ======================================================
        # dt
        # ======================================================
        if timestamp is None:

            dt = self.default_dt

        elif self.prev_time is None:

            dt = self.default_dt

        else:

            dt = float(
                timestamp
                - self.prev_time
            )

            if (
                not np.isfinite(dt)
                or dt < 1e-4
                or dt > 0.1
            ):

                dt = (
                    self.default_dt
                )

        if timestamp is not None:

            self.prev_time = float(
                timestamp
            )

        # ======================================================
        # Foot raw positions
        # ======================================================
        (
            l_point,
            l_contact_z,
        ) = self._get_foot_point(
            out,
            "Left",
        )

        (
            r_point,
            r_contact_z,
        ) = self._get_foot_point(
            out,
            "Right",
        )

        # ======================================================
        # Ground estimate
        # ======================================================
        ground_candidate = min(
            l_contact_z,
            r_contact_z,
        )

        if self.ground_z is None:

            self.ground_z = (
                ground_candidate
            )

        else:

            # --------------------------------------------------
            # 只允许近地面的脚更新 ground，
            # 防止双脚抬高时 ground 跟着上升。
            # --------------------------------------------------
            if (
                ground_candidate
                < self.ground_z + 0.03
            ):

                alpha_ground = 0.01

                self.ground_z = (
                    (1.0 - alpha_ground)
                    * self.ground_z
                    + alpha_ground
                    * ground_candidate
                )

        # ======================================================
        # Raw foot velocity
        #
        # 注意：
        # 这里必须使用 correction 前的 raw foot。
        # ======================================================
        vel = {}

        for side, point in (
            ("Left", l_point),
            ("Right", r_point),
        ):

            prev = (
                self.prev_point[side]
            )

            if prev is None:

                v = np.zeros(
                    3,
                    dtype=np.float64,
                )

            else:

                v = (
                    point - prev
                ) / dt

            vel[side] = v

            self.prev_point[side] = (
                point.copy()
            )

        # ======================================================
        # Contact detection
        # ======================================================
        (
            lc,
            l_enter,
            l_exit,
            l_zrel,
            l_vxy,
            l_vz,
        ) = self._update_contact(
            "Left",
            l_point,
            l_contact_z,
            vel["Left"],
        )

        (
            rc,
            r_enter,
            r_exit,
            r_zrel,
            r_vxy,
            r_vz,
        ) = self._update_contact(
            "Right",
            r_point,
            r_contact_z,
            vel["Right"],
        )

        # ======================================================
        # Step / anchor debug defaults
        # ======================================================
        left_anchor_updated = False
        right_anchor_updated = False

        left_step_dist = 0.0
        right_step_dist = 0.0

        left_step_reason = "none"
        right_step_reason = "none"

        # ======================================================
        # New Left contact -> gated anchor update
        # ======================================================
        if l_enter:

            (
                left_anchor_updated,
                left_step_dist,
                left_step_reason,
            ) = self._update_anchor_on_enter(
                "Left",
                l_point,
            )

        # ======================================================
        # New Right contact -> gated anchor update
        # ======================================================
        if r_enter:

            (
                right_anchor_updated,
                right_step_dist,
                right_step_reason,
            ) = self._update_anchor_on_enter(
                "Right",
                r_point,
            )

        # ======================================================
        # Active support selection
        # ======================================================
        (
            previous_support,
            active_support,
            support_switched,
        ) = self._select_active_support(
            lc=lc,
            rc=rc,

            l_enter=l_enter,
            r_enter=r_enter,

            l_vxy=l_vxy,
            r_vxy=r_vxy,

            l_vz=l_vz,
            r_vz=r_vz,
        )

        # ======================================================
        # Desired X correction
        #
        # 只使用 active support。
        # ======================================================
        desired_x = None

        if (
            active_support == "Left"
            and lc
            and self.anchor_xy["Left"]
            is not None
        ):

            desired_x = (
                self.anchor_xy[
                    "Left"
                ][0]
                - l_point[0]
            )

        elif (
            active_support == "Right"
            and rc
            and self.anchor_xy["Right"]
            is not None
        ):

            desired_x = (
                self.anchor_xy[
                    "Right"
                ][0]
                - r_point[0]
            )

        # ======================================================
        # Smooth X correction
        # ======================================================
        if desired_x is not None:

            # --------------------------------------------------
            # First-order low-pass
            # --------------------------------------------------
            alpha = (
                1.0
                - np.exp(
                    -dt
                    / max(
                        self.correction_tau,
                        1e-4,
                    )
                )
            )

            correction_error_x = (
                desired_x
                - self.correction_xy[0]
            )

            step_x = (
                alpha
                * correction_error_x
            )

            # --------------------------------------------------
            # Rate limit
            #
            # 默认：
            # 0.12 m/s @ 100 Hz
            # ≈ 1.2 mm/frame
            # --------------------------------------------------
            max_step = (
                self.max_correction_rate
                * dt
            )

            step_x = float(
                np.clip(
                    step_x,
                    -max_step,
                    max_step,
                )
            )

            self.correction_xy[0] += (
                step_x
            )

        # ======================================================
        # Safety clamp
        # ======================================================
        self.correction_xy[0] = float(
            np.clip(
                self.correction_xy[0],
                -self.max_correction,
                self.max_correction,
            )
        )

        # ======================================================
        # V2:
        # Y correction completely disabled
        # ======================================================
        self.correction_xy[1] = 0.0

        # ======================================================
        # Apply SAME X correction to every human target
        # ======================================================
        for name in out.keys():

            out[name][0][0] += (
                self.correction_xy[0]
            )

            # --------------------------------------------------
            # V2 deliberately disables Y correction.
            # --------------------------------------------------
            # out[name][0][1] += self.correction_xy[1]

        # ======================================================
        # Debug anchor values
        # ======================================================
        left_anchor_x = (
            float(
                self.anchor_xy["Left"][0]
            )
            if self.anchor_xy["Left"]
            is not None
            else np.nan
        )

        right_anchor_x = (
            float(
                self.anchor_xy["Right"][0]
            )
            if self.anchor_xy["Right"]
            is not None
            else np.nan
        )

        # ======================================================
        # Debug
        # ======================================================
        debug = {
            # --------------------------------------------------
            # Contact
            # --------------------------------------------------
            "left_contact": bool(lc),
            "right_contact": bool(rc),

            "left_enter": bool(l_enter),
            "right_enter": bool(r_enter),

            "left_exit": bool(l_exit),
            "right_exit": bool(r_exit),

            # --------------------------------------------------
            # Active support
            # --------------------------------------------------
            "previous_support": previous_support,
            "active_support": active_support,

            "support_switched": bool(
                support_switched
            ),

            # --------------------------------------------------
            # Correction
            # --------------------------------------------------
            "desired_x": (
                float(desired_x)
                if desired_x
                is not None
                else np.nan
            ),

            "corr_x": float(
                self.correction_xy[0]
            ),

            "corr_y": 0.0,

            # --------------------------------------------------
            # Anchor
            # --------------------------------------------------
            "left_anchor_x": (
                left_anchor_x
            ),

            "right_anchor_x": (
                right_anchor_x
            ),

            # --------------------------------------------------
            # Step gate
            # --------------------------------------------------
            "left_step_event": bool(
                l_enter
            ),

            "right_step_event": bool(
                r_enter
            ),

            "left_step_dist": float(
                left_step_dist
            ),

            "right_step_dist": float(
                right_step_dist
            ),

            "left_anchor_updated": bool(
                left_anchor_updated
            ),

            "right_anchor_updated": bool(
                right_anchor_updated
            ),

            "left_step_reason": (
                left_step_reason
            ),

            "right_step_reason": (
                right_step_reason
            ),

            # --------------------------------------------------
            # Contact detector diagnostics
            # --------------------------------------------------
            "left_z_rel": float(
                l_zrel
            ),

            "right_z_rel": float(
                r_zrel
            ),

            "left_vxy": float(
                l_vxy
            ),

            "right_vxy": float(
                r_vxy
            ),

            "left_vz": float(
                l_vz
            ),

            "right_vz": float(
                r_vz
            ),

            "ground_z": float(
                self.ground_z
            ),

            # --------------------------------------------------
            # Config useful for logs
            # --------------------------------------------------
            "step_update_thresh": float(
                self.step_update_thresh
            ),

            "max_correction": float(
                self.max_correction
            ),
        }

        return out, debug