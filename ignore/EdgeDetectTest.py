import cv2
import tkinter as tk
from tkinter import Scale
from PIL import Image, ImageTk

# Function to update brightness threshold and edge detection when slider values change
def update_images(min_thresh_val, max_thresh_val, param1_val, param2_val):
    min_thresh_val_float = float(min_thresh_val)
    max_thresh_val_float = float(max_thresh_val)
    
    _, thresh_img = cv2.threshold(img_gray, min_thresh_val_float, max_thresh_val_float, cv2.THRESH_BINARY)
    img_thresh_rgb = cv2.cvtColor(thresh_img, cv2.COLOR_GRAY2RGB)
    img_tk_thresh = ImageTk.PhotoImage(image=Image.fromarray(img_thresh_rgb))
    panel_thresh.configure(image=img_tk_thresh)
    panel_thresh.image = img_tk_thresh

    edges = cv2.Canny(img_gray, param1_val, param2_val)
    edges_rgb = cv2.cvtColor(edges, cv2.COLOR_GRAY2RGB)
    img_with_edges = cv2.addWeighted(img_rgb, 0.7, edges_rgb, 0.3, 0)
    img_tk = ImageTk.PhotoImage(image=Image.fromarray(img_with_edges))
    panel_edges.configure(image=img_tk)
    panel_edges.image = img_tk


# Load the image
img = cv2.imread("whiteboardWithTextEx.jpg")
img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2GRAY)

# Create a tkinter window
root = tk.Tk()
root.title("Image Processing")

# Create sliders for adjusting brightness threshold (min and max)
min_brightness_slider = Scale(root, from_=0, to=255, orient="horizontal", label="Min Brightness Threshold",
                              command=lambda val: update_images(val, max_brightness_slider.get(),
                                                               param1_slider.get(), param2_slider.get()))
min_brightness_slider.pack()
min_brightness_slider.set(0)  # initial value

max_brightness_slider = Scale(root, from_=0, to=255, orient="horizontal", label="Max Brightness Threshold",
                              command=lambda val: update_images(min_brightness_slider.get(), val,
                                                               param1_slider.get(), param2_slider.get()))
max_brightness_slider.pack()
max_brightness_slider.set(255)  # initial value

# Create sliders for adjusting edge detection parameters (param1 and param2)
param1_slider = Scale(root, from_=0, to=200, orient="horizontal", label="Param1",
                      command=lambda val: update_images(min_brightness_slider.get(), max_brightness_slider.get(),
                                                         int(val), param2_slider.get()))
param1_slider.pack()
param1_slider.set(50)  # initial value

param2_slider = Scale(root, from_=0, to=200, orient="horizontal", label="Param2",
                      command=lambda val: update_images(min_brightness_slider.get(), max_brightness_slider.get(),
                                                         param1_slider.get(), int(val)))
param2_slider.pack()
param2_slider.set(150)  # initial value

# Initial brightness thresholding and edge detection
_, initial_thresh_img = cv2.threshold(img_gray, min_brightness_slider.get(), max_brightness_slider.get(),
                                      cv2.THRESH_BINARY)
img_thresh_rgb_initial = cv2.cvtColor(initial_thresh_img, cv2.COLOR_GRAY2RGB)
img_tk_thresh_initial = ImageTk.PhotoImage(image=Image.fromarray(img_thresh_rgb_initial))

edges_initial = cv2.Canny(img_gray, param1_slider.get(), param2_slider.get())
edges_rgb_initial = cv2.cvtColor(edges_initial, cv2.COLOR_GRAY2RGB)
img_with_edges_initial = cv2.addWeighted(img_rgb, 0.7, edges_rgb_initial, 0.3, 0)
img_tk_edges_initial = ImageTk.PhotoImage(image=Image.fromarray(img_with_edges_initial))

# Display the original image with brightness threshold
panel_thresh = tk.Label(root, image=img_tk_thresh_initial)
panel_thresh.pack(side="left")

# Display the image with edges
panel_edges = tk.Label(root, image=img_tk_edges_initial)
panel_edges.pack(side="right")

root.mainloop()
