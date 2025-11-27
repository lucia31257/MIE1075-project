from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import uvicorn
import base64
from pathlib import Path
from ros_bridge import get_bridge

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class AudioData(BaseModel):
    audio: str  # Base64 编码的音频
    format: str = "webm"

@app.post("/speech_to_text")
async def speech_to_text(data: AudioData):
    print("Received audio data (Base64)")
    
    try:
        # 解码 Base64
        if "," in data.audio:
            base64_audio = data.audio.split(",")[1]  # 移除 "data:audio/webm;base64," 前缀
        else:
            base64_audio = data.audio
        
        audio_bytes = base64.b64decode(base64_audio)
        print(f"Decoded {len(audio_bytes)} bytes")
        
        # 保存到临时文件
        temp_path = f"/tmp/audio_{id(data)}.{data.format}"
        with open(temp_path, "wb") as f:
            f.write(audio_bytes)
        
        print(f"Saved to: {temp_path}")
        
        # 调用 ROS2
        bridge = get_bridge()
        text = bridge.call_speech_to_text(temp_path)
        
        # 清理临时文件
        Path(temp_path).unlink(missing_ok=True)
        
        return {"text": text}
    
    except Exception as e:
        print(f"Error: {str(e)}")
        import traceback
        traceback.print_exc()
        return {"text": f"Error: {str(e)}"}

@app.get("/")
async def root():
    return {"message": "FastAPI + ROS2 Base64 Server"}

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)