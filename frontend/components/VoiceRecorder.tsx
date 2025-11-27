"use client";

import { useState, useRef } from "react";

export default function VoiceRecorder({ onResult }) {
  const [isRecording, setIsRecording] = useState(false);
  const [isProcessing, setIsProcessing] = useState(false);
  const mediaRecorderRef = useRef<MediaRecorder | null>(null);
  const chunksRef = useRef<Blob[]>([]);

  const startRecording = async () => {
    const stream = await navigator.mediaDevices.getUserMedia({ audio: true });
    
    chunksRef.current = [];
    
    const recorder = new MediaRecorder(stream);
    mediaRecorderRef.current = recorder;

    recorder.ondataavailable = (event) => {
      chunksRef.current.push(event.data);
    };

    recorder.onstop = async () => {
      const blob = new Blob(chunksRef.current, { type: "audio/webm" });
      
      // 转换为 Base64
      const reader = new FileReader();
      reader.readAsDataURL(blob);
      reader.onloadend = async () => {
        const base64Audio = reader.result as string;
        
        setIsProcessing(true);
        try {
          const res = await fetch("http://localhost:8000/speech_to_text", {
            method: "POST",
            headers: {
              "Content-Type": "application/json",
            },
            body: JSON.stringify({
              audio: base64Audio,
              format: "webm"
            }),
          });
          
          const data = await res.json();
          onResult(data.text || "No text recognized");
        } catch (err) {
          console.error("Error:", err);
          onResult("Error contacting server");
        } finally {
          setIsProcessing(false);
        }
      };
      
      // 停止所有音频轨道
      stream.getTracks().forEach(track => track.stop());
    };

    recorder.start();
    setIsRecording(true);
  };

  const stopRecording = () => {
    mediaRecorderRef.current?.stop();
    setIsRecording(false);
  };

  return (
    <div className="flex flex-col items-center space-y-2">
      <button
        onClick={isRecording ? stopRecording : startRecording}
        disabled={isProcessing}
        className={`px-6 py-3 rounded-lg text-white text-lg transition ${
          isRecording 
            ? "bg-red-600 animate-pulse" 
            : isProcessing
            ? "bg-gray-400 cursor-not-allowed"
            : "bg-blue-600"
        } hover:opacity-90`}
      >
        {isProcessing ? "⏳ Processing..." : isRecording ? "⛔ Stop Recording" : "🎤 Start Recording"}
      </button>
      {isRecording && (
        <p className="text-sm text-gray-600 animate-pulse">
          Recording... Click to stop
        </p>
      )}
    </div>
  );
}
