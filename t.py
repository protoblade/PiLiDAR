import asyncio
import websockets
from lib.config import Config
from lib.lidar_driver import Lidar


clients = set()

target_resolution = 0.5

config = Config()
config.init()
config.update_target_res(target_resolution)



lidar = Lidar(config)

async def bcast():
    for ws in clients:
        try:
            await ws.send(lidar.points_2d.dumps())
        except:
            pass

async def broadcast():
    try:
        lidar.read_loop(callback=bcast, max_packages=config.max_packages)

    except KeyboardInterrupt:
        print("KeyboardInterrupt: Stopping read loop.")
        lidar.close()
        print("Exiting program.")

async def handler(websocket, path):
    clients.add(websocket)
    try:
        await websocket.wait_closed()
    finally:
        clients.remove(websocket)

async def main():
    async with websockets.serve(handler, '', config.WEBSOCKET_PORT):
        await broadcast()

if __name__ == "__main__":
    asyncio.run(main())








# callback function for lidar.read_loop()
