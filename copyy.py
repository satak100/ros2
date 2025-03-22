import tkinter as tk
import speech_recognition as sr
import threading
import os

# Function to start the recording process
def start_recording():
    def record_audio():
        recognizer = sr.Recognizer()
        mic = sr.Microphone()

        with mic as source:
            while is_recording:
                print("Listening...")  # You can add this for debugging purposes
                try:
                    audio = recognizer.listen(source)
                    text = recognizer.recognize_google(audio)
                    print(f"Recognized text: {text}")  # For debugging purposes

                    # Save the recognized text to a file
                    with open("C:\\shared\\audio_input.txt", "a") as file:
                        file.write(text + "\n")
                except sr.UnknownValueError:
                    # If speech is unintelligible, continue listening
                    continue
                except sr.RequestError as e:
                    print(f"Error with the speech recognition service: {e}")
                    break

    # Start the recording in a separate thread
    global is_recording
    is_recording = True
    recording_thread = threading.Thread(target=record_audio)
    recording_thread.daemon = True
    recording_thread.start()

# Function to stop the recording process
def stop_recording():
    global is_recording
    is_recording = False
    print("Recording stopped.")

# Create the main window
root = tk.Tk()
root.title("Voice-to-Text Recorder")

# Button to start recording
start_button = tk.Button(root, text="Start Recording", command=start_recording)
start_button.pack(pady=10)

# Button to stop recording
stop_button = tk.Button(root, text="Stop Recording", command=stop_recording)
stop_button.pack(pady=10)

# Button to close the application
close_button = tk.Button(root, text="Exit", command=root.quit)
close_button.pack(pady=10)

# Variable to control the recording status
is_recording = False

# Start the GUI event loop
root.mainloop()
