from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from google import genai
from dotenv import load_dotenv
from module.prompt import qa_instruction, guide_instruction, imagen_instruction
from module.tts import text_to_audio_base64
from google.genai import types
import os
import base64
from PIL import Image
from fastapi import WebSocket, WebSocketDisconnect
import json
import io
import uvicorn
load_dotenv()
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
client = genai.Client(api_key=GEMINI_API_KEY)

app = FastAPI()

# QA ChatBot
class QA_ChatRequest(BaseModel):
    message: str 

qa_history = [{'role': 'model', 'parts': [{'text': qa_instruction}]}]
qa_chat = client.chats.create(model="gemini-2.5-pro", history=qa_history)

@app.post("/qa_bot/init")
def init_chat():
    try:
        qa_history = [{'role': 'model', 'parts': [{'text': qa_instruction}]}]
        qa_chat = client.chats.create(model="gemini-2.5-pro", history=qa_history)
        return {
            "message": "New chat session initialized.",
            "history": qa_chat.get_history()
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to initialize chat: {str(e)}")

@app.post("/qa_bot/send_message")
def send_message(request: QA_ChatRequest):
    try:
        prompt = request.message
        response = qa_chat.send_message(prompt)
        return {
            "message": response.text,
            "history": qa_chat.get_history()
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to send message: {str(e)}")

# Guide ChatBot 
class Steps(BaseModel):
    step_number: int 
    step_description: str 
    img_description: str 

def generate_step(content: str):
    response = client.models.generate_content(
        model = "gemini-2.5-flash",
        contents = content, 
        config = types.GenerateContentConfig(
            system_instruction = guide_instruction,
            response_mime_type = "application/json",
            response_schema = list[Steps],)
    ) 
    return response.parsed 


@app.websocket("/ws/guide_bot/send_message")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            data = await websocket.receive_text()
            data_json = json.loads(data)
            prompt = data_json["message"]
            response = generate_step(prompt)

            for res in response:
                step_number = res.step_number
                step_description = res.step_description
                img_description = res.img_description

                # Tạo prompt ảnh
                img_prompt = "Instructional image of: " + img_description
                print(f"img_prompt: {img_prompt}")

                # Gọi model tạo ảnh (bạn thay bằng logic thực)
                response_imagen = client.models.generate_images(
                    model='imagen-4.0-generate-preview-06-06',
                    prompt=img_prompt,
                    config=types.GenerateImagesConfig(
                        number_of_images=1,
                    )
                )
                img_bytes = response_imagen.generated_images[0].image.image_bytes
                img_base64_str = base64.b64encode(img_bytes).decode('utf-8')
                # Get audio in base64
                audio_base64 = text_to_audio_base64(step_description)
                # Gửi từng bước qua WebSocket
                await websocket.send_json({
                    "step_number": step_number,
                    "step_description": step_description,
                    "img_base64": img_base64_str,
                    "audio_base64": audio_base64
                })

            await websocket.send_json({"status": "done"})

    except WebSocketDisconnect:
        print("Client disconnected")


# ASR ChatBot 
class ASR_ChatRequest(BaseModel):
    message: str  # base64 encoded audio

@app.post("/asr_bot/send_message")
def send_message(request: ASR_ChatRequest):
    # Decode base64 -> bytes
    audio_bytes = base64.b64decode(request.message)

    response = client.models.generate_content(
    model='gemini-2.5-flash',
    contents=[
        'Transcribe this audio clip into Vietnamese',
        types.Part.from_bytes(
        data=audio_bytes,
        mime_type='audio/wav',
        )
    ]
    )

    print(response.text)
    return {"text": response.text}

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8989)