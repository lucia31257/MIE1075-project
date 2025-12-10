"use client";

import { useState } from "react";

export default function TextPage() {
  const [inputText, setInputText] = useState("");
  const [isLoading, setIsLoading] = useState(false);
  const [response, setResponse] = useState<{
    status?: string;
    message?: string;
    query?: string;
    error?: string;
  } | null>(null);
  const [result, setResult] = useState<any>(null);

  const handleSubmit = async () => {
    if (!inputText.trim()) {
      alert("Please enter some text");
      return;
    }

    setIsLoading(true);
    setResponse(null);

    try {
      const res = await fetch("http://localhost:8000/send_query", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({
          text: inputText.trim(),
        }),
      });

      const data = await res.json();

      if (res.ok) {
        setResponse(data);
        setInputText(""); // 清空输入框
      } else {
        setResponse({ error: data.detail || "Failed to send query" });
      }
    } catch (error) {
      console.error("Error:", error);
      setResponse({
        error: "Network error. Please check if the server is running.",
      });
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === "Enter" && !e.shiftKey) {
      e.preventDefault();
      handleSubmit();
    }
  };

  return (
    <main className="flex flex-col items-center justify-center min-h-screen space-y-6 bg-gray-50 p-6">
      <h1 className="text-2xl font-bold text-gray-800">Search Query Input</h1>

      {/* Input Box */}
      <div className="w-full max-w-2xl space-y-4">
        <textarea
          value={inputText}
          onChange={(e) => setInputText(e.target.value)}
          onKeyPress={handleKeyPress}
          placeholder="Enter your search query (e.g., 'find the milk')"
          className="w-full p-4 border-2 border-gray-300 rounded-lg focus:border-blue-500 focus:outline-none resize-none"
          rows={4}
          disabled={isLoading}
        />

        <button
          onClick={handleSubmit}
          disabled={isLoading || !inputText.trim()}
          className={`w-full py-3 px-6 rounded-lg font-semibold text-white transition-colors
            ${
              isLoading || !inputText.trim()
                ? "bg-gray-400 cursor-not-allowed"
                : "bg-blue-600 hover:bg-blue-700"
            }`}
        >
          {isLoading ? "Sending..." : "Send Query"}
        </button>
      </div>

      {/* Response Display */}
      {response && (
        <div
          className={`w-full max-w-2xl p-4 rounded-lg ${
            response.error
              ? "bg-red-50 border-2 border-red-300"
              : "bg-green-50 border-2 border-green-300"
          }`}
        >
          {response.error ? (
            <div>
              <p className="font-semibold text-red-800">Error:</p>
              <p className="text-red-700">{response.error}</p>
            </div>
          ) : (
            <div>
              <p className="font-semibold text-green-800">Success!</p>

              <p className="text-sm text-gray-600 mt-2">
                Query sent: <span className="font-mono">{response.query}</span>
              </p>
            </div>
          )}
        </div>
      )}

      {/* Results */}
      {result && (
        <div className="w-full max-w-2xl p-4 bg-green-50 border-2 border-green-300 rounded-lg">
          <p className="font-semibold text-green-800 mb-2">Search Results:</p>
          <pre className="text-sm text-gray-700 overflow-auto">
            {JSON.stringify(result, null, 2)}
          </pre>
        </div>
      )}
    </main>
  );
}
