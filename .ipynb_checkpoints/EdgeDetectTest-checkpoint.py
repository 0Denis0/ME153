import cv2
import tkinter as tk
from tkinter import Scale
from PIL import Image, ImageTk

# Function to update edge detection when slider values change
def update_edges(param1_val, param2_val):
    edges = cv2.Canny(img_gray, param1_val, param2_val)
    edges_rgb = cv2.cvtColor(edges, cv2.COLOR_GRAY2RGB)
    img_with_edges = cv2.addWeighted(img_rgb, 0.7, edges_rgb, 0.3, 0)
    img_tk = ImageTk.PhotoImage(image=Image.fromarray(img_with_edges))
    panel.configure(image=img_tk)
    panel.image = img_tk

# Load the image
try:
    img = cv2.imread("whiteboardWithTextEx.jpg")
    if img is None:
        raise FileNotFoundError("Unable to load image.")
except Exception as e:
    print("Error:", e)
    exit()

img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2GRAY)

# Create a tkinter window
root = tk.Tk()
root.title("Edge Detection")

# Create sliders for adjusting parameters
param1_slider = Scale(root, from_=0, to=200, orient="horizontal", label="Param1",
                      command=lambda val: update_edges(int(val), param2_slider.get()))
param1_slider.pack()
param1_slider.set(50)  # initial value

param2_slider = Scale(root, from_=0, to=200, orient="horizontal", label="Param2",
                      command=lambda val: update_edges(param1_slider.get(), int(val)))
param2_slider.pack()
param2_slider.set(150)  # initial value

# Initial edge detection
edges_initial = cv2.Canny(img_gray, param1_slider.get(), param2_slider.get())
edges_rgb_initial = cv2.cvtColor(edges_initial, cv2.COLOR_GRAY2RGB)
img_with_edges_initial = cv2.addWeighted(img_rgb, 0.7, edges_rgb_initial, 0.3, 0)
img_tk_initial = ImageTk.PhotoImage(image=Image.fromarray(img_with_edges_initial))

# Display the image with edges
panel = tk.Label(root, image=img_tk_initial)
panel.pack()

root.mainloop()
