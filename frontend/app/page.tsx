import Link from "next/link";

export default function Home() {
  return (
    <main className="flex h-screen flex-col items-center justify-center space-y-6 bg-gray-50">
      <h1 className="text-3xl font-bold text-gray-800">
        Shop Assistant Robot
      </h1>

      <div className="flex space-x-4">
        <Link
          href="/voice"
          className="px-6 py-3 rounded-lg bg-blue-600 text-white text-lg hover:bg-blue-700 transition"
        >
           Voice Input
        </Link>

        <Link
          href="/text"
          className="px-6 py-3 rounded-lg bg-green-600 text-white text-lg hover:bg-green-700 transition"
        >
           Text Input
        </Link>
      </div>
    </main>
  );
}
