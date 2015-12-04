import json

import cv2
import numpy as np

r = {
    'cameras': {},
    'shots': {}
}

calibration_words = open('bin/calibration.txt').read().split()

fx, fy, cx, cy, k1, k2, p1, p2 = map(float, calibration_words[:8])
width, height = map(int, calibration_words[8:10])
size = max(width, height)

r['cameras']['slamcam'] = {
    'width': width,
    'height': height,
    'focal': np.sqrt(fx * fy) / size,
    'k1': k1,
    'k2': k2
}


keyframe_words = open('bin/reconstruction.txt').read().split()

nframes = int(keyframe_words[0])

for i in range(1, len(keyframe_words), 14):
    words = keyframe_words[i:i + 14]

    keyframe_id = int(words[0])
    frame_id = int(words[1])
    print frame_id
    P = np.array([float(w) for w in words[2:14]]).reshape(3, 4)
    R = P[:, :3]
    t = P[:, 3] * 10
    r['shots']['frame{:06d}.png'.format(frame_id)] = {
        'camera': 'slamcam',
        'rotation': list(cv2.Rodrigues(R)[0].flat),
        'translation': list(t.flat)
    }

with open('reconstruction.json', 'w') as fout:
    json.dump([r], fout, indent=4)
