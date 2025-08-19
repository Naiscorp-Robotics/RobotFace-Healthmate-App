from fastapi import FastAPI
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
import os
import uvicorn
app = FastAPI()

# Mount thư mục static để serve các file tĩnh (html, css, js, hình ảnh)
app.mount("/static", StaticFiles(directory="static"), name="static")

@app.get("/", response_class=HTMLResponse)
async def root():
    with open("static/test_websocket.html", "r", encoding="utf-8") as f:
        html_content = f.read()
    return HTMLResponse(content=html_content, status_code=200)
if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=9898)