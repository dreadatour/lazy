#!/usr/bin/python
# -*- coding: utf-8 -*-
import datetime
import time
import math
import os

import serial
import cv


CAM = 0
PORT = '/dev/tty.usbserial-A800eznq'
BAUD = 9600

CROP_X = 290
CROP_Y = 130
CROP_W = 270
CROP_H = 120
LCD_W = 227
LCD_H = 67


class Lazy:
    def __init__(self):
        # init servo
        self.servo = serial.Serial(port=PORT, baudrate=BAUD)
        # init cam
        self.cam = cv.CaptureFromCAM(CAM)

    def save_file(self, img, suffix=None):
        if suffix is None:
            suffix = ''
        else:
            suffix = "-%s" % suffix
        now = datetime.datetime.now()
        filename = "img/%s%s.png" % (now.strftime("%Y%m%dT%H%M%S"), suffix)
        cv.SaveImage(filename, img)

    def get_frame(self):
        # get frame
        frame = cv.QueryFrame(self.cam)

        # crop frame
        cv.SetImageROI(frame, (CROP_X, CROP_Y, CROP_H, CROP_W))
        raw = cv.CreateImage(cv.GetSize(frame), frame.depth, frame.channels)
        cv.Copy(frame, raw)
        cv.ResetImageROI(frame)

        # rotate frame - only LCD needed
        lcd = cv.CreateImage((raw.height, raw.width), raw.depth, raw.channels)
        cv.Transpose(raw, lcd)
        cv.Flip(lcd, lcd, flipMode=1)

        # filtering
        cv.Smooth(lcd, lcd, cv.CV_GAUSSIAN, 3, 0)

        return lcd

    def dots_dist(self, p0, p1):
        return (p0[0] - p1[0]) ** 2 + (p0[1] - p1[1]) ** 2

    def line_angle(self, line):
        a = float(line[1][1] - line[0][1]) / float(line[1][0] - line[0][0])
        return 180 / math.pi * math.atan(a)

    def max_line(self, l1, l2=None):
        if l2 is None:
            return l1

        d1 = self.dots_dist(l1[0], l2[0])
        d2 = self.dots_dist(l1[0], l2[1])
        d3 = self.dots_dist(l1[1], l2[0])
        d4 = self.dots_dist(l1[1], l2[1])
        d5 = self.dots_dist(l1[0], l1[1])
        d6 = self.dots_dist(l2[0], l2[1])

        if d1 >= max(d2, d3, d4, d5, d6):
            return [l1[0], l2[0]]
        elif d2 >= max(d1, d3, d4, d5, d6):
            return [l1[0], l2[1]]
        elif d3 >= max(d1, d2, d4, d5, d6):
            return [l1[1], l2[0]]
        elif d4 >= max(d1, d2, d3, d5, d6):
            return [l1[1], l2[1]]
        elif d5 >= max(d1, d2, d3, d4, d6):
            return l1
        else:
            return l2

    def def_line(self, l):
        a = l[0][1] - l[1][1]
        b = l[1][0] - l[0][0]
        c = (l[0][0] - l[1][0]) * l[0][1] + (l[1][1] - l[0][1]) * l[0][0]
        return (a, b, c)

    def cross_lines(self, l1, l2):
        d = l1[0] * l2[1] - l2[0] * l1[1]
        if d == 0:
            return None
        x = (l2[2] * l1[1] - l1[2] * l2[1]) / d
        y = (l2[0] * l1[2] - l1[0] * l2[2]) / d
        return (x, y)

    def rotate_image(self, image, angle, around=None):
        if around is None:
            around = (0, 0)
        rotate = cv.CreateImage(cv.GetSize(image), image.depth, 1)
        rotate = cv.CloneImage(image)

        mapping = cv.CreateMat(2, 3, cv.CV_32FC1)
        cv.GetRotationMatrix2D(around, angle, 1, mapping)
        cv.WarpAffine(image, rotate, mapping)

        return rotate

    def cut_image(self, image, x, y, w, h):
        cv.SetImageROI(image, (x, y, w, h))
        result = cv.CreateImage(cv.GetSize(image), image.depth, image.channels)
        cv.Copy(image, result)
        cv.ResetImageROI(image)
        return result

    def get_lcd(self):
        frame = self.get_frame()

        # convert the image to grayscale
        grey = cv.CreateImage(cv.GetSize(frame), cv.IPL_DEPTH_8U, 1)
        cv.CvtColor(frame, grey, cv.CV_RGB2GRAY)

        # canny
        canny = cv.CreateImage(cv.GetSize(frame), cv.IPL_DEPTH_8U, 1)
        cv.Canny(grey, canny, 200, 255, 3)

        # find contours
        storage = cv.CreateMemStorage(0)
        contours = cv.FindContours(canny, storage, cv.CV_RETR_LIST,
                cv.CV_CHAIN_APPROX_SIMPLE)
        if len(contours) < 2:
            return None

        # parse contours - find lines
        ht, hb, vl, vr = None, None, None, None
        prev_point = contours[-1]
        for point in contours:
            if self.dots_dist(prev_point, point) > 36:
                l = [list(prev_point), list(point)]
                if abs(l[0][0] - l[1][0]) > abs(l[0][1] - l[1][1]):
                    if max(l[0][1], l[1][1]) < CROP_H / 2:
                        ht = self.max_line(l, ht)
                    else:
                        hb = self.max_line(l, hb)
                else:
                    if max(l[0][0], l[1][0]) < CROP_W / 2:
                        vl = self.max_line(l, vl)
                    else:
                        vr = self.max_line(l, vr)
            prev_point = point

        if not ht or not hb or not vl or not vr:
            return None

        # find corners
        lt = self.cross_lines(self.def_line(ht), self.def_line(vl))
        rt = self.cross_lines(self.def_line(ht), self.def_line(vr))
        lb = self.cross_lines(self.def_line(hb), self.def_line(vl))
        rb = self.cross_lines(self.def_line(hb), self.def_line(vr))

        # check size and geometry
        w1 = math.sqrt(self.dots_dist(lt, rt))
        w2 = math.sqrt(self.dots_dist(lb, rb))
        h1 = math.sqrt(self.dots_dist(lt, lb))
        h2 = math.sqrt(self.dots_dist(rt, rb))
        d1 = math.sqrt(self.dots_dist(lt, rb))
        d2 = math.sqrt(self.dots_dist(rt, lb))

        # optimal width is around 227
        if 222 < w1 < 232 and 222 < w2 < 232:
            w = LCD_W  # int((w1 + w2) / 2)
        else:
            return None

        # optimal height is around 67
        if 62 < h1 < 72 and 62 < h2 < 72:
            h = LCD_H  # int((h1 + h2) / 2)
        else:
            return None

        # diagonals must be equals
        if abs(d1 - d2) > 10:
            return None

        # rotate and cut lcd image
        angle = self.line_angle(ht)
        grey = self.rotate_image(grey, angle, lt)
        grey = self.cut_image(grey, lt[0] + 1, lt[1] + 1, w, h)

        return grey

    def get_diff(self, img1, img2):
        diff = cv.CreateImage(cv.GetSize(img1), img1.depth, 1)
        diff = cv.CloneImage(img1)
        cv.AbsDiff(img1, img2, diff)
        return diff

    def threshold(self, img, edge):
        result = cv.CreateImage(cv.GetSize(img), img.depth, 1)
        cv.Threshold(img, result, edge, 255, cv.CV_THRESH_BINARY)
        cv.Erode(result, result, None, 1)
        return result

    def count_pixels(self, img, x, y, w, h):
        result = 0
        for yy in range(y, y + h):
            for xx in range(x, x + w):
                result += img[yy, xx]
        return result

    def parse_digit(self, img):
        LIM = 1000
        digits = {
            '1110111': 0,
            '0010010': 1,
            '1011101': 2,
            '1011011': 3,
            '0111010': 4,
            '1101011': 5,
            '1101111': 6,
            '1110010': 7,
            '1111111': 8,
            '1111011': 9
        }
        c1 = self.count_pixels(img, 10,  0, 7, 7) > LIM and 1 or 0
        c2 = self.count_pixels(img,  0, 12, 7, 7) > LIM and 1 or 0
        c3 = self.count_pixels(img, 20, 12, 7, 7) > LIM and 1 or 0
        c4 = self.count_pixels(img, 10, 24, 7, 7) > LIM and 1 or 0
        c5 = self.count_pixels(img,  0, 36, 7, 7) > LIM and 1 or 0
        c6 = self.count_pixels(img, 20, 36, 7, 7) > LIM and 1 or 0
        c7 = self.count_pixels(img, 10, 48, 7, 7) > LIM and 1 or 0
        res = "%d%d%d%d%d%d%d" % (c1, c2, c3, c4, c5, c6, c7)
        if res in digits:
            return digits[res]
        return None

    def run(self):
        step = 0
        empty = None
        results = []
        result = None

        while True:
            step += 1

            if empty is None:
                if step > 30:
                    os.system('/usr/local/bin/growlnotify -t "Токен" -m "Не могу обнаружить токен"')
                    break

                empty = self.get_lcd()
                if empty is None:
                    continue

                self.servo.write('G')
                time.sleep(1)
                continue

            if step > 100:
                os.system('/usr/local/bin/growlnotify -t "Токен" -m "Не могу распознать цифры"')
                break

            lcd = self.get_lcd()
            if lcd is None:
                continue

            diff = self.get_diff(lcd, empty)

            for n in range(1, 255):
                img = self.threshold(diff, n)
                right = self.count_pixels(img, 170, 0, LCD_W - 170, LCD_H)
                if right == 0:
                    diff = self.threshold(diff, n + 5)
                    break

            numbers = ''
            for n in range(0, 6):
                cv.SetImageROI(diff, (5 + n * 27, 7, 27, 55))
                d = cv.CreateImage(cv.GetSize(diff), diff.depth, diff.channels)
                cv.Copy(diff, d)
                d = self.rotate_image(d, 3)
                cv.Dilate(d, d, None, 2)
                number = self.parse_digit(d)
                cv.ResetImageROI(diff)

                if number is None:
                    break
                numbers = "%s%s" % (numbers, number)

            if len(numbers) != 6:
                continue

            results.append(numbers)
            if len(results) < 2:
                continue

            if results[-1] == results[-2]:
                result = results[-1]
                break

        if result is not None:
            os.system('/usr/local/bin/growlnotify -t "Токен" -m "Скопировано: %s"' % result)
            pb = os.popen("pbcopy", "w")
            pb.write(result)
            pb.close()

if __name__ == "__main__":
    t = Lazy()
    time.sleep(1)
    t.run()
