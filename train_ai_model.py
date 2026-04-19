import os
import pandas as pd
import numpy as np
import tensorflow as tf
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler

# ==============================================================================
# 1. LOAD AND PREPARE MULTI-CLASS DATASET
# ==============================================================================
# Define the files and their target classes:
# Class 0: Empty Room (Noise / Ghost Tracks)
# Class 1: Active People (Moving around)
# Class 2: Unconscious (Stationary, purely vital signs)
DATA_FILES = {
    'empty.csv': 0,
    'moving.csv': 1,
    'unconscious.csv': 2
}

# The ESP32 outputs 13 columns (the 13th being the C Heuristic label). 
# We DO care about the ESP32's heuristic label now! We will use it to clean the dataset.
base_columns = ['Prefix', 'TID', 'posX', 'posY', 'posZ', 'velX', 'velY', 'velZ', 'conf', 'HR', 'BR', 'BR_dev', 'Heuristic']

df_list = []
for filename, label_id in DATA_FILES.items():
    if not os.path.exists(filename):
        print(f"Warning: {filename} not found. Skipping...")
        continue
        
    print(f"Loading {filename} as Class {label_id}...")
    
    # Read all 13 columns
    df_temp = pd.read_csv(filename, names=base_columns, on_bad_lines='skip')
    
    if label_id == 1:
        # For 'moving', the radar might still report a ghost track (like a fan) in the corner.
        # The C-level Heuristic gate will have flagged that ghost with a '0' in the 13th column.
        # So we MUST drop any row in this dataset where the C-code thought it was a ghost to prevent poisoning.
        num_before = len(df_temp)
        df_temp = df_temp[df_temp['Heuristic'] == 1]
        print(f"  -> Cleaned {(num_before - len(df_temp))} heuristic ghosts from {filename}")
    else:
        # If it's the 'empty.csv' (Class 0) or 'unconscious.csv' (Class 2), keep everything.
        # - For empty, we want to train on even "strong" ghosts to recognize them as empty.
        # - For unconscious, the person is completely still, so their heuristic score might drop to 0, but we still want to learn their vital signs.
        pass
        
    df_temp['Label'] = label_id
    df_list.append(df_temp)

if not df_list:
    print("Error: No data files found! Please record empty.csv, moving.csv, and unconscious.csv")
    print("Example: pio device monitor | grep 'CSV_TRACK' > empty.csv")
    exit(1)

df = pd.concat(df_list, ignore_index=True)

# Drop irrelevant text columns and the C Heuristic Flag (we only needed it to clean)
df = df.drop(columns=['Prefix', 'TID', 'Heuristic'])

# Drop corrupted rows
df = df.dropna()

# Extract Features (X) and Labels (y)
X = df.drop(columns=['Label']).values
y = df['Label'].values.astype(int)

# Split dataset
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# Normalize the data (Crucial for Neural Networks)
scaler = StandardScaler()
X_train_scaled = scaler.fit_transform(X_train)
X_test_scaled = scaler.transform(X_test)

print(f"\nDataset Ready! Training samples: {len(X_train)}, Testing samples: {len(X_test)}")
print(f"Classes found: {np.unique(y)}")

# ==============================================================================
# 2. BUILD AND TRAIN MULTI-CLASS DEEP LEARNING MODEL
# ==============================================================================
NUM_CLASSES = 3

model = tf.keras.Sequential([
    tf.keras.layers.Dense(16, activation='relu', input_shape=(X_train.shape[1],)),
    tf.keras.layers.Dense(8, activation='relu'),
    # Softmax output layer for multi-class probabilities (outputs array of 3 probabilities)
    tf.keras.layers.Dense(NUM_CLASSES, activation='softmax') 
])

model.compile(optimizer='adam',
              loss='sparse_categorical_crossentropy',  # Required for integer multi-class labels
              metrics=['accuracy'])

print("\nTraining Multi-Class Neural Network...")
model.fit(X_train_scaled, y_train, epochs=50, batch_size=16, validation_data=(X_test_scaled, y_test))

loss, accuracy = model.evaluate(X_test_scaled, y_test, verbose=0)
print(f"\nModel Test Accuracy: {accuracy * 100:.2f}%")

# ==============================================================================
# 3. EXPORT TO TENSORFLOW LITE (TFLite Micro)
# ==============================================================================
print("\nConverting to TFLite format for ESP32-S3...")
converter = tf.lite.TFLiteConverter.from_keras_model(model)
converter.optimizations = [tf.lite.Optimize.DEFAULT]
tflite_model = converter.convert()

with open("rescue_vision_model.tflite", "wb") as f:
    f.write(tflite_model)

# ==============================================================================
# 4. GENERATE C HEADER FILE FOR ESP-IDF
# ==============================================================================
print("Generating C header file (include/rescue_vision_model.h)...")

hex_array = ', '.join([f'0x{byte:02x}' for byte in tflite_model])
mean_str = ', '.join([f'{m:.6f}' for m in scaler.mean_])
scale_str = ', '.join([f'{s:.6f}' for s in scaler.scale_])

c_model_array = f"""#pragma once
// Automatically generated by train_ai_model.py
// ESP32-S3 TFLite Micro Model for Multi-Class Target Classification

// CLASSES:
// 0 = Empty Room / Ghost Noise
// 1 = Active Moving Person
// 2 = Unconscious Person 

#include <stdint.h>

const unsigned int rescue_vision_model_len = {len(tflite_model)};
const unsigned char rescue_vision_model[] = {{
    {hex_array}
}};

// -------------------------------------------------------------
// STANDARD SCALER PARAMETERS (Must apply exactly before inference)
// For each feature: scaled_x = (x - mean) / scale
// -------------------------------------------------------------
const float feature_means[10] = {{{mean_str}}};
const float feature_scales[10] = {{{scale_str}}};

// Feature Order: posX, posY, posZ, velX, velY, velZ, conf, HR, BR, BR_dev
"""

with open("include/rescue_vision_model.h", "w") as f:
    f.write(c_model_array)

print("\nDone! -> include/rescue_vision_model.h generated successfully!")
