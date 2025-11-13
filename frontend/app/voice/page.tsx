"use client";

import { useState } from "react";
import VoiceRecorder from "@/components/VoiceRecorder";

export default function VoicePage() {
  const [recognizedText, setRecognizedText] = useState("");

  return (
    <main className="flex flex-col items-center justify-center min-h-screen space-y-6 bg-gray-50 p-6">
      <h1 className="text-2xl font-bold text-gray-800">🎤 Voice Input</h1>

      {/* comp */}
      <VoiceRecorder onResult={(text: string) => setRecognizedText(text)} />

      <div className="w-full max-w-xl p-4 border rounded-lg bg-white shadow">
        <h2 className="text-lg font-semibold mb-2">Recognized Text:</h2>
        <p className="text-gray-700 min-h-[60px]">{recognizedText || "–"}</p>
      </div>
    </main>
  );
}
