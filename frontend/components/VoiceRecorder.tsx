"use client";

import { useState, useRef } from "react";

export default function VoiceRecorder({ onResult }) {
  const [isRecording, setIsRecording] = useState(false);
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

      const formData = new FormData();
      formData.append("file", blob, "audio.webm");

      try {
        const res = await fetch("http://localhost:8000/speech_to_text", {
          method: "POST",
          body: formData,
        });

        const data = await res.json();

        onResult(data.text);
      } catch (err) {
        console.error("Error:", err);
        onResult("Error contacting server");
      }
    };

    recorder.start();
    setIsRecording(true);
  };

  const stopRecording = () => {
    mediaRecorderRef.current?.stop();
    setIsRecording(false);
  };

  return (
    <button
      onClick={isRecording ? stopRecording : startRecording}
      className={`px-6 py-3 rounded-lg text-white text-lg transition ${
        isRecording ? "bg-red-600" : "bg-blue-600"
      } hover:opacity-90`}
    >
      {isRecording ? "⛔ Stop Recording" : "🎤 Start Recording"}
    </button>
  );
}
