/*
Copyright 2025 人形机器人（上海）有限公司, https://www.openloong.net
Thanks for the open biped control project Nabo: https://github.com/tryingfly/nabo

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

============ ***doc description @ yyp*** ============

=====================================================*/
#include<string>
#include<chrono>
#include<thread>
#include"mujoco/mujoco.h"
#include"glfw3.h"

using namespace std;
namespace Sim{

void init(const string& sceneFile,void(*keyboardFunc)(GLFWwindow* window, int key, int scancode, int act, int mods));
void reset();
void runStep();
void rander(const char*msg);
void close();
mjModel &getMjModel();
mjData &getMjData();
GLFWwindow &getWindow();

}//namespace