#!/bin/bash

echo "🚀  Starting Sync Process..."

# 1. Push to GitHub
echo "---------------------------------"
echo "📦  Pushing to GitHub..."
git add .
git commit -m "Auto-save update"
git push origin main

# 2. Sync to Google Drive (Excluding the dangerous .git folder)
echo "---------------------------------"
echo "☁️   Syncing to Google Drive..."
# rclone sync [Source Folder] [RemoteName]:[Target Folder]
rclone sync . gdrive:DroneProjectFolder --exclude ".git/**" --exclude "__pycache__/**" --progress

echo "---------------------------------"
echo "✅  All Done! Ready for Colab."
