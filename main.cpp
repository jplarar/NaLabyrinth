/*
 * CopyrightSensor (c) 2012-2014 Aldebaran Robotics. All rightSensors reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */
#include <iostream>
#include <alproxies/alsonarproxy.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/allandmarkdetectionproxy.h>
#include <alproxies/altexttospeechproxy.h>
#include <alproxies/alnavigationproxy.h>
#include <alproxies/alvideodeviceproxy.h>
#include <alvision/alimage.h>
#include <alvision/alvisiondefinitions.h>
#include <alcommon/albroker.h>
#include <unistd.h>
#include <math.h>
#include <cmath>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <numeric>
#include <algorithm>




#define _USE_MATH_DEFINES //PI
#define distToBack 0.28 //Distance to the wall to stop at
#define distToWall 0.35 //Distance to the wall to stop at
#define distToTurn 0.5 //Distance to the wall to stop at
#define clearPathDistance 0.5
#define bumperDistance 0.7
#define spin90rightSensor ((-M_PI/2)+0.26) // Turn rightSensor PI/2 Radians +- error
#define spin90leftSensor ((M_PI/2)-0.17)// Turn leftSensor PI/2 Radians +- error
#define spin180leftSensor (M_PI-0.52) //30 grados

using namespace std;
using namespace cv; // Opencv namespace
using namespace AL; // Aldebaran namespace

//Variables globales

//Variables globales utilizadas para guardar la distancia regresada por los sensores
//ultrasonicsVariables used to store the data from the sensors
float leftSensor = 0.0;
float rightSensor = 0.0;

/*
*  updadateUS
*
*  Metodo encargado de actualizar las distancias proporcionadas por los sensores
*  ultrasonicos
*
*  @param
*/
void updateUS(AL::ALMemoryProxy memory){

    //leftSensor = memory.getData("Device/SubDeviceList/US/Left/Sensor/Value");
    //rightSensor = memory.getData("Device/SubDeviceList/US/Right/Sensor/Value");
    vector<float>leftSensorLectures;
    vector<float>rightSensorLectures;

    //Obtiene 5 nuevas lecturas y las almacena en un vector
    for (int i=0; i < 7; i++){
        leftSensorLectures.push_back(memory.getData("Device/SubDeviceList/US/Left/Sensor/Value"));
                          rightSensorLectures.push_back(memory.getData("Device/SubDeviceList/US/Right/Sensor/Value"));
      sleep(0.1);
    }
    sort(leftSensorLectures.begin(),leftSensorLectures.end());
    sort(rightSensorLectures.begin(),rightSensorLectures.end());
    leftSensor = leftSensorLectures.at(3);
    rightSensor = rightSensorLectures.at(3);
    leftSensorLectures.clear();
    rightSensorLectures.clear();
}

/*
*  align
*
*  Metodo encargado de alinealeftSensorr el robot nao para garantizar un caminado recto
*
*  @param
*/
bool align(AL::ALMotionProxy movement, AL::ALMemoryProxy memory, AL::ALTextToSpeechProxy say)
{
    float diffSensor = fabs(rightSensor-leftSensor);
    cout<<diffSensor<<" leftSensor:"<<leftSensor<<" rightSensor:"<<rightSensor<<endl;
    //Se ajusta la posicion mientras una diferencia entre el sensor izquierdo y derecho
    float minSensor = min(rightSensor, leftSensor);
    cout<<"min:"<<minSensor<<endl;
    float percentage = diffSensor/minSensor;
    cout<<"percentage:"<<percentage<<endl;
    if (percentage > .2 && (rightSensor < clearPathDistance || leftSensor < clearPathDistance)) {

        diffSensor = fabs(rightSensor-leftSensor);
        if (movement.moveIsActive()) {
             movement.stopMove();
        }
        say.post.say("Align");
        cout<<"Ajustar posicion"<<" leftSensor:"<<leftSensor<<" rightSensor:"<<rightSensor<<endl;
        //
        if (rightSensor < leftSensor) {
            cout<<"Gira Iquierda"<<endl;
            //movement.moveTo(-0.1,0,M_PI/7*percentage);
            movement.move(-0.05,0,M_PI/20);
        } else {
            cout<<"Gira rightSensor"<<endl;
            //movement.moveTo(-0.1,0,-M_PI/7*percentage);
            movement.move(-0.05,0,-M_PI/20);
        }
        while(percentage > .04){
            updateUS(memory);
            diffSensor = fabs(rightSensor-leftSensor);
            minSensor = min(rightSensor, leftSensor);
            percentage = diffSensor/minSensor;
        }
        movement.stopMove();
        //movement.waitUntilMoveIsFinished();
        return true;
    }
    return false;
}

/*
*  changeDirection
*
*  description
*
*  @param
*/
void changeDirection(AL::ALMotionProxy movement, AL::ALMemoryProxy memory, AL::ALTextToSpeechProxy say)
{
    // gira rightSensor
    say.post.say("Turn right");
    movement.moveTo(0,0,spin90rightSensor);
    updateUS(memory);
    if (leftSensor < distToTurn && rightSensor < distToTurn) {
        // gira leftSensor
        if (leftSensor < distToBack || rightSensor < distToBack) {
            movement.stopMove();
            say.post.say("Too close");
            movement.moveTo(-0.05,0,0);
        }
        say.post.say("Turn left");
        movement.moveTo(0,0,spin180leftSensor);
        updateUS(memory);
        if (leftSensor < distToBack || rightSensor < distToBack) {
            movement.stopMove();
            say.post.say("Too close");
            movement.moveTo(-0.05,0,0);
        }
        if (leftSensor < distToTurn && rightSensor < distToTurn) {
            // regresa por donde llego
            say.post.say("Wrong way");
            movement.moveTo(0,0,spin90leftSensor);
        }
    }
}

bool bumperPressed(AL::ALMotionProxy movement, AL::ALMemoryProxy memory, AL::ALTextToSpeechProxy say)
{
    float bumperRight = memory.getData("RightBumperPressed");
    float bumperLeft = memory.getData("LeftBumperPressed");
    if (bumperRight > 0 && leftSensor > bumperDistance && rightSensor > bumperDistance ) {
        movement.stopMove();
        say.post.say("Ouch right");
        movement.moveTo(-0.25,0,M_PI/16);
        movement.moveTo(0,0.1,0);
        return true;
    }
    if (bumperLeft > 0 && leftSensor > bumperDistance && rightSensor > bumperDistance) {
        movement.stopMove();
        say.post.say("Ouch left");
        movement.moveTo(-0.25,0,-M_PI/16);
        movement.moveTo(0,-0.1,0);
        return true;
    }
    return false;
}

void searchNaomark(AL::ALVideoDeviceProxy camProxy, AL::ALMotionProxy movimiento, AL::ALMemoryProxy memoria, AL::ALTextToSpeechProxy say)
{
    camProxy.setActiveCamera(0); //Connect to top camera
    usleep(1000000);             //Wait for the switch of camera.

    int contador;
    bool detected ;

    AL::ALValue markInfo = "";

    contador = 0;
    detected = false;

    //Check if there is a Naomark in front of the robot,do not check the sides.
    do{
        markInfo = memoria.getData("LandmarkDetected");
        contador++;
        if (markInfo.getSize()!=0)
            detected = true;
    }while(contador < 10 && !detected);

    if(markInfo.getSize() != 0) {
        std::string markID = "";
        markID = markInfo[1][0][1].toString().substr(1,3);
        say.post.say(markID);
        std::cout << "markID = " << markID << std::endl;
    }
    else
    {
        std::cout << "No se detectó marca enfrente" << std::endl;

        //Turn Nao's head left to check if there is a Naomark,if there is, the Nao will turn left.
        /* ************************************************************************************** */
        movimiento.angleInterpolation("HeadYaw",M_PI/2, 1.0 ,true);

        usleep(1000000);

        contador = 0;
        detected = false;
        do{
            markInfo = memoria.getData("LandmarkDetected");
            contador++;
            if (markInfo.getSize()!=0)
                detected = true;
        }while(contador < 10 && !detected);

        if(markInfo.getSize() != 0) {
            std::string markID = "";
            markID = markInfo[1][0][1].toString().substr(1,3);
            say.post.say(markID);
            std::cout << "markID = " << markID << std::endl;

            //Turn left
            movimiento.moveTo(0,0, spin90leftSensor-0.21);
        }
        else
        {
            //Turn Nao's head right to check if there is a Naomark to the right, if there is, the Nao will turn right.
            /* ************************************************************************************** */

            std::cout << "No se detectó marca a la izquierda" << std::endl;
            movimiento.angleInterpolation("HeadYaw",-M_PI/2, 1.0 ,true);

            usleep(1000000);

            contador = 0;
            detected = false;
            do{
                markInfo = memoria.getData("LandmarkDetected");
                contador++;
                if (markInfo.getSize()!=0)
                    detected = true;
            }while(contador < 10  && !detected);

            if(markInfo.getSize() != 0) {
                std::string markID = "";
                markID = markInfo[1][0][1].toString().substr(1,3);
                say.post.say(markID);
                std::cout << "markID = " << markID << std::endl;

                //Turn Right
                movimiento.moveTo(0,0, spin90rightSensor);
            }
            else
            {
                std::cout << "No se detectó a la derecha" << std::endl;
            }
        }

        //Turn the head to the front
        movimiento.angleInterpolation("HeadYaw",0, 1.0 ,true);
    }
}

/*
*  main
*
*  description
*
*  @param
*/
int main(int argc, char *argv[])
{
    String robotIP =argv[1];
    int port =9559;
    AL::ALSonarProxy sonar(robotIP,port);
    AL::ALRobotPostureProxy posture(robotIP,port);
    AL::ALMotionProxy movement(robotIP,port);
    AL::ALLandMarkDetectionProxy naoMark(robotIP,port);
    AL::ALTextToSpeechProxy say(robotIP, port);
    AL::ALMemoryProxy memory(robotIP, port);
    AL::ALVideoDeviceProxy camProxy(robotIP, port);
    sonar.subscribe("ALSonar");
    bool stand;

    // Inicializar postura
    std::string  actualPosture;
    stand = posture.goToPosture("Stand",1);

    //tiempo de actualizacion de marca en memory
    int period = 500;
    // Inicializar deteccion de marcas
    naoMark.subscribe("Test_Mark", period, 0.0);

    Mat src, src_gray;

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    //TEST
    updateUS(memory);

    //initial
    say.post.say("Start");

    while(1)
    {
        actualPosture = posture.getPostureFamily();
        // update sonar
        updateUS(memory);
        // Check Bumpers
        //searchNaomark(camProxy,movement, memory, say);
        //Stand up and align with the wall on its left if the robot was on the floor.
        if ((actualPosture != "Standing")) {
            posture.goToPosture("Stand",1);
        } else if (bumperPressed(movement, memory, say)) {

        } else if (rightSensor < distToBack && rightSensor < distToBack) {
            movement.stopMove();
            say.post.say("Too close");
            movement.moveTo(-0.15,0,0);
        } else if (rightSensor < distToBack) {
            movement.stopMove();
            say.post.say("Too close Right");
            movement.moveTo(-0.15,0,M_PI/10);
        } else if (leftSensor < distToBack) {
            movement.stopMove();
            say.post.say("Too close Left");
            movement.moveTo(-0.15,0,-M_PI/10);
        } else if(align(movement, memory, say)) {
        } else if (leftSensor <= distToWall && rightSensor <= distToWall) {
            movement.stopMove();
            say.post.say("Change Direction");
            changeDirection(movement, memory, say);
        } else if (!movement.moveIsActive()){
            say.post.say("Start walking");
           movement.move(0.1,0,0);
        }

    }

    return 0;
}
