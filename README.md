# Database
- Use SQLite
## How to update the database
Edit the csv files in `src/shop_assistant_robot/resource`. If the schema changed(add/change/delete columns), update the schema in `src/shop_assistant_robot/script/init_db.py` line 15-32.
# Test the connect from ui to ros
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

# frontend
## Prerequisites
### Node.js
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
