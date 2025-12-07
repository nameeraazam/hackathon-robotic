# Week 13: Conversational AI - Speech-to-Text with Whisper

This guide explains how to integrate a powerful speech-to-text (STT) model, OpenAI's Whisper, into a ROS 2 application. This will allow our robot to understand voice commands from a user.

## 1. What is OpenAI's Whisper?

Whisper is a state-of-the-art automatic speech recognition (ASR) model developed by OpenAI. It was trained on a massive and diverse dataset of audio from the internet, making it incredibly robust against accents, background noise, and technical language.

### Key Features:
- **High Accuracy:** It achieves human-level performance on many benchmarks.
- **Multilingual:** It can transcribe audio in dozens of languages.
- **Robustness:** Performs well even with significant background noise.
- **Open Source:** The model and its code are open source, allowing you to run it locally.

For our Butler Bot project, Whisper is an excellent choice for converting the user's spoken command (e.g., "Bot, bring me the red can") into text that our robot's AI can process.

## 2. System Design

We will create a ROS 2 node that:
1.  Listens for raw audio data from a microphone.
2.  When it detects speech, it captures an audio clip.
3.  Sends this audio clip to the Whisper model for transcription.
4.  Publishes the resulting text to a ROS 2 topic.

## 3. Installation and Setup

### Step 3.1: Install Whisper

First, you need to install the `openai-whisper` Python package. You will also need `ffmpeg` for audio processing.

```bash
# Install ffmpeg system-wide
sudo apt update
sudo apt install ffmpeg

# Install the whisper package via pip
pip install openai-whisper
```

Depending on your hardware, you may also need to install PyTorch with CUDA support to get the best performance.

### Step 3.2: Setting up an Audio Source in ROS 2

You need a ROS 2 node that can publish microphone data. A common way to do this is using the `audio_common` package.

```bash
# Install the audio_common package for your ROS distro
sudo apt install ros-humble-audio-common
```

You can then run the `audio_capture` node to publish microphone data to the `/audio` topic.

```bash
# Launch the microphone capture node
ros2 run audio_capture audio_capture_node
```
This will publish messages of type `audio_common_msgs/AudioData`.

## 4. Creating the Whisper ROS 2 Node

Now, let's create the Python node that will perform the transcription.

Create a new file `whisper_node.py` in your ROS 2 package.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData

import whisper
import numpy as np
import torch

# Check if CUDA is available
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"

class WhisperNode(Node):

    def __init__(self):
        super().__init__('whisper_node')
        
        # Load the Whisper model
        self.get_logger().info('Loading Whisper model...')
        self.model = whisper.load_model("base.en", device=DEVICE) # Use "base.en" for English
        self.get_logger().info('Whisper model loaded.')

        # Create subscriber to the audio topic
        self.audio_subscriber = self.create_subscription(
            AudioData,
            '/audio',
            self.audio_callback,
            10)
        
        # Create publisher for the transcribed text
        self.text_publisher = self.create_publisher(String, '/transcribed_text', 10)

        # Buffer to store audio data
        self.audio_buffer = []
        self.get_logger().info('Whisper node has started.')

    def audio_callback(self, msg):
        # The audio data is a list of bytes. We need to convert it to a NumPy array.
        # Assuming 16-bit signed integer format, which is common.
        audio_np = np.frombuffer(msg.data, dtype=np.int16)
        
        # Convert to float32, which is what Whisper expects
        audio_float32 = audio_np.astype(np.float32) / 32768.0
        
        # For simplicity, we process every incoming message.
        # A more advanced implementation would use Voice Activity Detection (VAD)
        # to buffer audio only when someone is speaking.
        
        self.get_logger().info('Transcribing audio...')
        try:
            # Transcribe
            result = self.model.transcribe(audio_float32, fp16=torch.cuda.is_available())
            transcribed_text = result['text']
            
            if transcribed_text:
                self.get_logger().info(f"Transcribed: {transcribed_text}")
                
                # Publish the transcribed text
                text_msg = String()
                text_msg.data = transcribed_text
                self.text_publisher.publish(text_msg)
                
        except Exception as e:
            self.get_logger().error(f"Error during transcription: {e}")


def main(args=None):
    rclpy.init(args=args)
    whisper_node = WhisperNode()
    rclpy.spin(whisper_node)
    whisper_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```

### How the Node Works:

1.  **Initialization:**
    -   It loads a pre-trained Whisper model. We use `base.en`, which is a small, fast, English-only model. For higher accuracy, you could use `medium.en` or `large`.
    -   It creates a subscriber to the `/audio` topic and a publisher for the `/transcribed_text` topic.

2.  **`audio_callback`:**
    -   This function is called every time a new `AudioData` message arrives.
    -   It converts the raw audio bytes into a NumPy array of floating-point numbers, which is the format Whisper requires.
    -   It calls `self.model.transcribe()` on the audio data.
    -   If the transcription is successful and returns text, it publishes that text as a `std_msgs/String` on the `/transcribed_text` topic.

## 5. Launching and Testing

To test the system, you need to run three components:
1.  The `audio_capture` node.
2.  Your `whisper_node`.
3.  A ROS 2 topic echo to listen for the output.

**Terminal 1: Audio Capture**
```bash
source /opt/ros/humble/setup.bash
ros2 run audio_capture audio_capture_node
```

**Terminal 2: Whisper Node**
```bash
# Make sure your ROS 2 package is built and sourced
source install/setup.bash
ros2 run your_package_name whisper_node
```

**Terminal 3: Listen for Text**
```bash
ros2 topic echo /transcribed_text
```

Now, speak into your microphone. You should see the `whisper_node` log that it is transcribing, and the final text should appear in Terminal 3. This text can now be used as an input to your robot's behavior tree or main logic controller.
