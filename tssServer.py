import asyncio
import websockets
import json
import base64
import os
from datetime import datetime

# Global variable to store connected clients
connected_clients = set()

async def echo(websocket):
    client_id = id(websocket)
    connected_clients.add(websocket)
    print(f"TSS Client {client_id} connected. Total clients: {len(connected_clients)}")
    
    try:
        async for message in websocket:
            print(f"TSS Received from client {client_id}: {message}")
            
            # Parse the received message
            try:
                data = json.loads(message)
                print(f"Parsed data: {data}")
                
                # Check if this is a command from the dashboard
                if data.get("type") == "dashboard_command":
                    await handle_dashboard_command(data, websocket)
                else:
                    # Handle regular client requests
                    await handle_client_request(data, websocket)
                    
            except json.JSONDecodeError:
                print("Invalid JSON received, sending error response")
                await websocket.send(json.dumps({
                    "error": "Invalid JSON format",
                    "received_message": message
                }))
                continue
            
    except websockets.ConnectionClosed:
        print(f"TSS Client {client_id} disconnected")
    finally:
        connected_clients.discard(websocket)
        print(f"TSS Client {client_id} removed. Total clients: {len(connected_clients)}")

async def handle_client_request(data, websocket):
    """Handle regular client requests"""
    step_number = data.get("step_number", 1)
    step_description = data.get("step_description", "Default step")
    
    # Generate sample image and audio data (base64 encoded)
    sample_image_data = "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVR42mNkYPhfDwAChwGA60e6kgAAAABJRU5ErkJggg=="
    sample_audio_data = "UklGRnoGAABXQVZFZm10IBAAAAABAAEAQB8AAEAfAAABAAgAZGF0YQoGAACBhYqFbF1fdJivrJBhNjVgodDbq2EcBj+a2/LDciUFLIHO8tiJNwgZaLvt559NEAxQp+PwtmMcBjiR1/LMeSwFJHfH8N2QQAoUXrTp66hVFApGn+DyvmwhBSuBzvLZiTYIG2m98OScTgwOUarm7blmGgU7k9n1unEiBC13yO/eizEIHWq+8+OWT"
    
    # Send structured response
    response_data = {
        "step_number": step_number,
        "step_description": step_description,
        "img_base64": sample_image_data,
        "audio_base64": sample_audio_data,
        "timestamp": asyncio.get_event_loop().time()
    }
    
    await websocket.send(json.dumps(response_data))
    print(f"TSS Sent structured response to client: {response_data}")

async def handle_dashboard_command(data, websocket):
    """Handle commands from the dashboard"""
    command = data.get("command")
    
    if command == "send_to_all_clients":
        # Send data to all connected clients
        step_data = data.get("step_data", {})
        await send_to_all_clients(step_data)
        # Send confirmation back to dashboard
        await websocket.send(json.dumps({
            "type": "dashboard_response",
            "status": "success",
            "message": f"Sent to {len(connected_clients)} clients",
            "timestamp": datetime.now().isoformat()
        }))
    
    elif command == "get_client_count":
        # Send client count back to dashboard
        await websocket.send(json.dumps({
            "type": "dashboard_response",
            "client_count": len(connected_clients),
            "timestamp": datetime.now().isoformat()
        }))

async def send_to_all_clients(step_data):
    """Send step data to all connected clients"""
    if not connected_clients:
        print("No clients connected to send data to")
        return
    
    # Prepare the step data
    step_number = step_data.get("step_number", 1)
    step_description = step_data.get("step_description", "Manual step from server")
    image_data = step_data.get("img_base64", "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVR42mNkYPhfDwAChwGA60e6kgAAAABJRU5ErkJggg==")
    audio_data = step_data.get("audio_base64", "UklGRnoGAABXQVZFZm10IBAAAAABAAEAQB8AAEAfAAABAAgAZGF0YQoGAACBhYqFbF1fdJivrJBhNjVgodDbq2EcBj+a2/LDciUFLIHO8tiJNwgZaLvt559NEAxQp+PwtmMcBjiR1/LMeSwFJHfH8N2QQAoUXrTp66hVFApGn+DyvmwhBSuBzvLZiTYIG2m98OScTgwOUarm7blmGgU7k9n1unEiBC13yO/eizEIHWq+8+OWT")
    
    message = {
        "step_number": step_number,
        "step_description": step_description,
        "img_base64": image_data,
        "audio_base64": audio_data,
        "timestamp": asyncio.get_event_loop().time(),
        "source": "server_manual"
    }
    
    # Send to all connected clients
    disconnected_clients = set()
    for client in connected_clients:
        try:
            await client.send(json.dumps(message))
            print(f"Sent manual step data to client {id(client)}")
        except websockets.ConnectionClosed:
            disconnected_clients.add(client)
        except Exception as e:
            print(f"Error sending to client {id(client)}: {e}")
            disconnected_clients.add(client)
    
    # Remove disconnected clients
    for client in disconnected_clients:
        connected_clients.discard(client)
    
    print(f"Sent step data to {len(connected_clients) - len(disconnected_clients)} clients")
    print(f"Removed {len(disconnected_clients)} disconnected clients")

async def main():
    async with websockets.serve(echo, "localhost", 12346):
        print("TSS WebSocket server started on ws://localhost:12346")
        print("Server can now manually send data to connected clients")
        print("Use the dashboard to send commands to the server")
        await asyncio.Future()  # run forever

if __name__ == "__main__":
    asyncio.run(main())
