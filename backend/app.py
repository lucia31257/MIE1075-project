from fastapi import FastAPI, UploadFile, File
from fastapi.middleware.cors import CORSMiddleware
import uvicorn

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
    fake_text = "I want to buy milk"
    return {"text": fake_text}

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
