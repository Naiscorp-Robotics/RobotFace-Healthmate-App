import asyncio
import websockets

async def echo(websocket):
    print("Client connected")
    try:
        async for message in websocket:
            print(f"Received: {message}")
            response = f"Server echo: {message}"
            await websocket.send(response)
    except websockets.ConnectionClosed:
        print("Client disconnected")

async def main():
    async with websockets.serve(echo, "localhost", 12345):
        print("WebSocket server started on ws://localhost:12345")
        await asyncio.Future()  # run forever

if __name__ == "__main__":
    asyncio.run(main())
