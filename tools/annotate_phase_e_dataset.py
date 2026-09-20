#!/usr/bin/env python3
"""Polygon annotation UI for Phase E Gazebo captures (standard-library only).

Click polygon vertices, choose a class, then Apply. Unpainted pixels stay 255
(IGNORE). Save writes frame_NNNN.mask.pgm. Keyboard: n/p frame, u undo, s save.
"""
import argparse
from pathlib import Path
import tkinter as tk

LABELS = [('BEDROCK', 0, '#afb9c8'), ('REGOLITH', 1, '#c29a5c'),
          ('ROCK', 2, '#eb4137'), ('CRATER', 3, '#7637a0'),
          ('SHADOW', 4, '#141824')]

class App:
    def __init__(self, root, directory):
        self.root, self.directory = root, directory
        self.frames = sorted(directory.glob('frame_*.ppm'))
        if not self.frames: raise SystemExit(f'no frame_*.ppm in {directory}')
        self.i = 0; self.label = 1; self.points = []; self.polygons = []
        bar = tk.Frame(root); bar.pack(fill='x')
        for name, value, color in LABELS:
            tk.Button(bar, text=f'{value} {name}', bg=color,
                      command=lambda v=value: setattr(self, 'label', v)).pack(side='left')
        for name, fn in [('Apply', self.apply), ('Undo', self.undo), ('Save', self.save),
                         ('Prev', lambda: self.move(-1)), ('Next', lambda: self.move(1))]:
            tk.Button(bar, text=name, command=fn).pack(side='left')
        self.status = tk.Label(root); self.status.pack(fill='x')
        self.canvas = tk.Canvas(root, cursor='cross'); self.canvas.pack()
        self.canvas.bind('<Button-1>', self.click)
        root.bind('s', lambda e: self.save()); root.bind('u', lambda e: self.undo())
        root.bind('n', lambda e: self.move(1)); root.bind('p', lambda e: self.move(-1))
        self.load()

    def load(self):
        self.image = tk.PhotoImage(file=str(self.frames[self.i]))
        self.w, self.h = self.image.width(), self.image.height()
        self.canvas.config(width=self.w, height=self.h); self.canvas.delete('all')
        self.canvas.create_image(0, 0, image=self.image, anchor='nw')
        self.mask = bytearray([255]) * (self.w * self.h); self.polygons = []; self.points = []
        mask = self.frames[self.i].with_suffix('.mask.pgm')
        if mask.exists(): self.mask[:] = read_pgm(mask, self.w, self.h)
        self.redraw(); self.status.config(text=f'{self.frames[self.i].name}  label={self.label}  IGNORE=255')

    def click(self, event):
        self.points.append((event.x, event.y)); self.redraw()
    def apply(self):
        if len(self.points) >= 3:
            fill_polygon(self.mask, self.w, self.h, self.points, self.label)
            self.polygons.append((self.points[:], self.label)); self.points = []; self.redraw()
    def undo(self):
        if self.points: self.points.pop()
        elif self.polygons:
            self.polygons.pop(); self.mask = bytearray([255]) * (self.w * self.h)
            for pts, lab in self.polygons: fill_polygon(self.mask, self.w, self.h, pts, lab)
        self.redraw()
    def redraw(self):
        self.canvas.delete('annotation')
        for pts, lab in self.polygons:
            color = LABELS[lab][2]; flat = [v for p in pts for v in p]
            self.canvas.create_polygon(*flat, outline=color, fill='', width=2, tags='annotation')
        if len(self.points) >= 2:
            flat = [v for p in self.points for v in p]
            self.canvas.create_line(*flat, fill='cyan', width=2, tags='annotation')
        elif len(self.points) == 1:
            x, y = self.points[0]
            self.canvas.create_oval(x-3, y-3, x+3, y+3, outline='cyan',
                                    fill='cyan', tags='annotation')
    def save(self):
        path = self.frames[self.i].with_suffix('.mask.pgm')
        path.write_bytes(f'P5\n{self.w} {self.h}\n255\n'.encode() + bytes(self.mask))
        counts = [self.mask.count(i) for i in range(5)]
        self.status.config(text=f'SAVED {path.name} counts={counts} ignore={self.mask.count(255)}')
    def move(self, delta):
        self.save(); self.i = (self.i + delta) % len(self.frames); self.load()

def read_pgm(path, w, h):
    data = path.read_bytes(); parts = data.split(b'\n', 3)
    if len(parts) != 4 or parts[0] != b'P5': raise ValueError(f'bad PGM {path}')
    body = parts[3]
    if len(body) != w*h: raise ValueError(f'mask shape mismatch {path}')
    return body

def fill_polygon(mask, w, h, points, label):
    # Even/odd scanline fill; annotation speed is sufficient for 640x480 frames.
    for y in range(max(0, min(p[1] for p in points)), min(h-1, max(p[1] for p in points))+1):
        xs = []
        for a, b in zip(points, points[1:] + points[:1]):
            if (a[1] <= y < b[1]) or (b[1] <= y < a[1]):
                xs.append(int(a[0] + (y-a[1])*(b[0]-a[0])/(b[1]-a[1])))
        xs.sort()
        for left, right in zip(xs[::2], xs[1::2]):
            for x in range(max(0,left), min(w-1,right)+1): mask[y*w+x] = label

def main():
    p=argparse.ArgumentParser(); p.add_argument('directory'); a=p.parse_args()
    root=tk.Tk(); root.title('LunaBot Phase E Gazebo Annotation'); App(root, Path(a.directory)); root.mainloop()
if __name__ == '__main__': main()
