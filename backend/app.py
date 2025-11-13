from fastapi import FastAPI, UploadFile, File
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
from ros_bridge import get_bridge


app = FastAPI()


app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.post("/speech_to_text")
async def speech_to_text(file: UploadFile = File(...)):
    print("Received file:", file.filename)  
    fake_text = "get"
    temp_name = "name"
    bridge = get_bridge()
    text = bridge.call_speech_to_text(temp_name)

    return {"text": f'fast:{fake_text},ros2:{text}'}

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
