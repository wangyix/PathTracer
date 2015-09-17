#include "st.h"
#include "stgl.h"
#include "stglut.h"
//#include "ExampleScene.h"
#include "Scene.h"
#include <iostream>
#include <time.h>

using namespace std;

int main(int argc, const char * argv[])
{
    std::string sceneFile = "./CornellBox.txt";     // default scenefile
    int blocks_x = 1;
    int blocks_y = 1;
    int block_i = 0;
    int block_j = 0;

    if (argc == 6) {
        sceneFile = argv[1];
        blocks_x = stoi(argv[2]);
        blocks_y = stoi(argv[3]);
        block_i = stoi(argv[4]);
        block_j = stoi(argv[5]);
    } else if (argc == 5) {
        blocks_x = stoi(argv[1]);
        blocks_y = stoi(argv[2]);
        block_i = stoi(argv[3]);
        block_j = stoi(argv[4]);
    } else if (argc == 2) {
        sceneFile = argv[1];
    } else if (argc == 1) {
    } else {
        cout << "Number of args not supported." << std::endl;
        exit(0);
    }

    if (blocks_x < 1 || blocks_y < 1
        || block_i < 0 || block_i >= blocks_x
        || block_j < 0 || block_j >= blocks_y) {
        cout << "Block args out of range!" << std::endl;
        exit(0);
    }

    Scene scene;
    scene.setRenderSubimage(blocks_x, blocks_y, block_i, block_j);
    scene.initializeSceneFromScript(sceneFile);

    //scene->initializeAs5NonUniformScene();
    //scene->initializeSceneFromScript("rtScene.txt");
    ////or set rendering scene from code
    ////scenes for assignment 4
    //scene->initializeSceneBasicGeometry();
    //scene->initializeSceneBasicLightingAndShading();
    //scene->initializeSceneTransform();
    //scene->initializeSceneObjMesh();
    //scene->initializeSceneObjMesh2();
    //scene->initializeSceneTexture();
    //scene->initializeSceneTransparentObject();
    //scene->initializeSceneTransparentObject2();

    ////scenes for assignment 5
    //scene->initializeSceneAccelerationStructureGrid();
    //scene->initializeSceneAccelerationStructureBVH();

    ////scenes for assignment 6
    //scene->initializeSceneDepthOfField();
    //scene->initializeSceneParticipatingMedia();

    ////set rendering scene from script files
    //scene->initializeSceneFromScript("../Standard_Tests/RecursiveTest.txt");
    //scene->initializeSceneFromScript("../Standard_Tests/CornellBox.txt");
    //scene->initializeSceneFromScript("../Standard_Tests/DoF.txt");
    //scene->initializeSceneFromScript("../Standard_Tests/Glass.txt");
    //scene->initializeSceneFromScript("../Standard_Tests/Go.txt");
    //scene->initializeSceneFromScript("../Standard_Tests/Bowl.txt");
    //scene->rtSampleRate(4);

    ////initialize acceleration structures
    //scene->buildAccelStructures(std::string("aabb"));
    //scene->buildAccelStructures(std::string("grid"));
    scene.buildAABBTrees();     // must call this!

    clock_t start, end;
    start=clock();

    scene.Render();

    end=clock();
    cout << "Render time: "<<(double)(end-start) / ((double)CLOCKS_PER_SEC)<<" s"<<std::endl;

    system("PAUSE");

    return 0;
}

