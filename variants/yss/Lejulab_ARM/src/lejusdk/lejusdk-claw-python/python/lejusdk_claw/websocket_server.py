import argparse
import asyncio
import json
from .transport import NativeTransport


async def serve(host, port):
    import websockets
    transport = NativeTransport()

    async def handler(connection, path=None):
        async for message in connection:
            try:
                payload = json.loads(message)
                # 只开放夹爪公共操作；阻塞等待放到线程，不占用 WebSocket 事件循环。
                if payload.get("op") not in ("state", "target", "control", "recover"):
                    raise ValueError("不支持的夹爪操作")
                data = await asyncio.get_running_loop().run_in_executor(None, transport.request, payload)
                result = {"ok": True, "data": data}
            except Exception as error:
                result = {"ok": False, "error": str(error)}
            await connection.send(json.dumps(result, allow_nan=False))

    try:
        async with websockets.serve(handler, host, port, max_size=65536):
            await asyncio.Future()
    finally:
        transport.disconnect()


def main():
    parser = argparse.ArgumentParser(description="夹爪 WebSocket 到 DDS SDK 适配服务")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    asyncio.run(serve(args.host, args.port))


if __name__ == "__main__":
    main()
