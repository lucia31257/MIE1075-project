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
---

# Database
- Use SQLite
## How to update the database
Edit the csv files in `src/shop_assistant_robot/resource`. If the schema changed(add/change/delete columns), update the schema in `src/shop_assistant_robot/script/init_db.py` line 15-32.

