#!/usr/bin/python3
import os

os.chdir(os.path.dirname(os.path.abspath(__file__)))
if os.path.exists("./../Core/Src/main.c"):
    os.rename("./../Core/Src/main.c", "./../Core/Src/main.cpp")