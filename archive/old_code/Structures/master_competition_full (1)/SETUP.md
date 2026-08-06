# Master Pi setup

## 1. System packages
    sudo apt-get install -y python3-pip portaudio19-dev libatomic1

## 2. Python packages
    pip3 install -r requirements.txt

## 3. Vosk model (offline speech recognition)
    wget https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip
    unzip vosk-model-small-en-us-0.15.zip

## 4. TLS cert (allows phone browser to use microphone over HTTPS)
    openssl req -x509 -newkey rsa:2048 -keyout key.pem -out cert.pem \
      -days 365 -nodes -subj "/CN=10.42.0.1"

## 5. Run
    sudo python3 app.py
    # Opens HTTPS on port 443
    # Phone browser: https://10.42.0.1
