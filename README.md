# Database
- Use SQLite
## How to update the database
Edit the csv files in `src/shop_assistant_robot/resource`. If the schema changed(add/change/delete columns), update the schema in `src/shop_assistant_robot/script/init_db.py` line 15-32.
# Test ui and ros voice rec node
1. start ros2 node
```
cd ros2-ws
source /opt/ros/humble/setup.bash
source ~/MIE1075-project/ros2-ws/install/setup.bash
ros2 run shop_assistant_robot speech_to_text_server
```
Output:
```
SpeechToText service ready
```
2. Start Fastapi service

Open another terminal

Install the dependencies if haven't.
```
pip install fastapi uvicorn python-multipart

```

```
cd ~/MIE1075-project/backend
source /opt/ros/humble/setup.bash
source ~/MIE1075-project/ros2-ws/install/setup.bash
python3 app.py
```
See
```
INFO:     Uvicorn running on http://0.0.0.0:8000
```
visit `http://0.0.0.0:8000/docs` to see the apis
3. Run frontend

## frontend
### Prerequisites
#### Node.js
- Node.js 18+
- npm

version check

```
node -v
npm -v
```
install node
```
curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
sudo apt install -y nodejs
```
## Start
```
cd frontend
npm install
npm run dev
```
go to `http://localhost:3000` to show the website
# voice rec ros node
## ros node test
### Prerequisites
```
sudo apt update && sudo apt install ffmpeg
pip install faster-whisper
```
### Test
Terminal 1: Start the node
```
cd ~/MIE1075-project/ros2-ws
source install/setup.bash
ros2 run shop_assistant_robot speech_to_text_server
```
Expected output
```
SpeechToText service ready.
```
Termial 2: pass in the voice file
```
ros2 service call /speech_to_text my_interfaces/srv/SpeechToText "{audio_path: '/tmp/test.wav'}"
```
real example 
```
source /opt/ros/humble/setup.bash
source ~/MIE1075-project/ros2-ws/install/setup.bash

if [ ! -f /tmp/test.wav ]; then
    echo "Downloading real speech sample..."
    cd /tmp
    if [ ! -d LibriSpeech ]; then
        wget -q --show-progress https://www.openslr.org/resources/12/test-clean.tar.gz
        tar -xzf test-clean.tar.gz
    fi
    
    TEST_FLAC=$(find LibriSpeech/test-clean/ -name "*.flac" | head -1)
    ffmpeg -i "$TEST_FLAC" -ar 16000 -ac 1 /tmp/test.wav -y -loglevel quiet
    echo "Created /tmp/test.wav"
fi

ls -lh /tmp/test*.wav

ros2 service call /speech_to_text my_interfaces/srv/SpeechToText "{audio_path: '/tmp/test.wav'}"
```
Expected to see
```
# Termial 2
response:
my_interfaces.srv.SpeechToText_Response(text='When the king comes to Paris, everybody calls out Vivalroix!')
```
```
# Termial 1
Transcribed text: When the king comes to Paris, everybody calls out Vivalroix!
```