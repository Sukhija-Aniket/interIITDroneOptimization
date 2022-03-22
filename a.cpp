#include<iostream>
#include<map>
#include<vector>
#include<bits/stdc++.h>
using namespace std;

// functions
int getSeconds(string time);


// item master
struct item{
    string itemId;
    int weight;
    int length,breadth,height;

    item(string Id,int wt,int len,int bth,int hgt){
        itemId = Id;
        weight = wt;
        length = len;
        breadth = bth;
        height = hgt;
    }
};

struct demand{
    int day;
    int demandId;
    int warehouseNumber;
    int itemId;
    int x,y,z;
    int deliveryStartTime,deliveryEndTime;
    bool deliveryFailure;

    demand(vector<string>row){
        demandId = stoi(row[1].substr(1));
        itemId = int(row[2][row[2].size() - 1]-'0') - 1;
        day = int(row[3][row[3].size()-1]-'0');
        warehouseNumber = int(row[0][row[0].size() - 1]-'0');
        x = stoi(row[4]);
        y = stoi(row[5]);
        z = stoi(row[6]);
        deliveryStartTime = getSeconds(row[7]);
        deliveryEndTime = getSeconds(row[8]);
        deliveryFailure = (row[9][0] - '0');
    }
};

struct speedProfile{
    int p,q;
};


struct energyConsumption{
    int a,b,c;
    int totalWeight;
    int speed;
    int altitudeGain;
};

struct drone{
    int droneType;
    int droneCount;
    int batteryCapacity;
    int baseWeight;
    int payloadCapacity;
    int volumePayloadCapacity;
    int maxSlots;
    int maxSpeed;
    int maintainanceFixCost;
    int maintainanceVariableCost;
    speedProfile pq;
    energyConsumption ec;

    drone(int dtype,int battery,int weight,int capacity,int volCapacity,int maxslots,int mfixcost,int mvarcost){
        droneType = dtype;
        batteryCapacity = battery;
        baseWeight = weight;
        payloadCapacity = capacity;
        volumePayloadCapacity = volCapacity;
        maxSlots = maxslots;
    }
};
struct singleDrone{
    int droneType;
    int index;
    int availableTime = 0;
    int remainingBattery;
    int nextAvailableTime;

    singleDrone(int dtype,int time,int battery,int indexCounter){
        droneType = dtype;
        availableTime = time;
        remainingBattery = battery;
        index = indexCounter;
    }

};


// struct costs{
//     int dronetype;
//     int maintainancefixcost;
//     int maintainancevariablecost;
// };

struct chargingInfo{
    string stationId;
    int chargeAllowed;
    int current;
    int xCoord,yCoord,zCoord=0;
    chargingInfo(string sId,int charge,int cur){
        stationId = sId;
        chargeAllowed = charge;
        current = cur;
    }
};

struct droneCostOutput{
    string droneId;
    string day;
    int restingTime;
    int ChargingTime;
    int MaintainanceCost;
    int EnergyCost;
};

struct dronePathOutput{
    int demandId;
    string droneId;
    string day;
    int x,y,z;
    string activity;
    int speed;
    double batteryConsumption;
    double EnergyCost;
    int totalWeight;
};

struct noFlyZone{
    int x[8];
    int y[8];
    int z[8];
};

// global variables
vector<item>items;
vector<demand>demands;
vector<singleDrone> singleDrones;
vector<drone>drones;

// utility functions
int getSeconds(string time){
    int hours = stoi(time.substr(0,2)) - 8;
    int minutes = stoi(time.substr(3,2));
    int seconds = stoi(time.substr(6,2));
    seconds += hours*3600 + minutes * 60;
    return seconds;
}

bool demandComparator(demand &a,demand &b){
    return a.deliveryEndTime < b.deliveryEndTime;
}

bool checkOptimalDrone(demand &curDemand,singleDrone &curDrone,vector<singleDrone>&validDrones){

    drone tempDrone = drones[curDrone.droneType];
    item tempItem = items[curDemand.itemId];
    int netWeight = tempItem.weight + tempDrone.baseWeight;

    //  cout<<"speedXY "<<tempDrone.maxSpeed<<" "<<tempDrone.pq.p* double(tempItem.weight)/double(tempDrone.payloadCapacity)<<" "<<double(tempDrone.payloadCapacity)<<endl;

    if(drones[curDrone.droneType].payloadCapacity < items[curDemand.itemId].weight) return false;
    if(tempDrone.volumePayloadCapacity < tempItem.length * tempItem.breadth * tempItem.height) return false;

    double netSpeedXY = tempDrone.maxSpeed - tempDrone.pq.p* double(tempItem.weight)/double(tempDrone.payloadCapacity);
    // cout<<"speedXY "<<tempDrone.maxSpeed<<" "<<tempDrone.pq.p* double(tempItem.weight)/double(tempDrone.payloadCapacity)<<" "<<double(tempDrone.payloadCapacity)<<endl;
    double netSpeedZUp = tempDrone.maxSpeed - tempDrone.pq.q * double(tempItem.weight)/double(tempDrone.payloadCapacity);
    double netSpeedReturn = tempDrone.maxSpeed;

    double batteryUsage = 0;
    // cout<<netSpeedXY<<" "<<netSpeedZUp<<endl;
    double distanceXY = sqrt(curDemand.x*curDemand.x + curDemand.y*curDemand.y);
    double distanceZ = curDemand.z;
    double timeUsed = distanceXY/netSpeedXY + distanceZ/netSpeedZUp;
    
    if(curDrone.availableTime + timeUsed + 360 > curDemand.deliveryEndTime) return false;
    double extraTime = curDrone.availableTime + timeUsed + 180;
    batteryUsage += netWeight * (tempDrone.ec.a + tempDrone.ec.b* (netSpeedXY)) * (distanceXY/netSpeedXY);
    batteryUsage += netWeight * (tempDrone.ec.a + tempDrone.ec.c*netSpeedZUp) * (distanceZ/netSpeedZUp);
    batteryUsage += tempDrone.baseWeight * (tempDrone.ec.a + tempDrone.ec.b* (netSpeedReturn)) * (distanceXY/netSpeedReturn);
    batteryUsage += tempDrone.baseWeight * (tempDrone.ec.a + tempDrone.ec.c* (netSpeedReturn)) * (distanceZ/netSpeedReturn);
    
    if(batteryUsage > curDrone.remainingBattery) return false;
    timeUsed += 360 + (distanceXY + distanceZ)/netSpeedReturn;
    // timeUsed += max(0.0,double((2000-(curDrone.remainingBattery - batteryUsage))*36)/50LL);
    timeUsed += double((tempDrone.batteryCapacity -(curDrone.remainingBattery-batteryUsage))*36)/50;
    
    singleDrone tempValidDrone = curDrone;
    timeUsed += max(0.0,curDemand.deliveryStartTime - extraTime);
    tempValidDrone.nextAvailableTime = timeUsed + curDrone.availableTime;
    // cout<<timeUsed<<" "<<curDrone.availableTime<<" "<<tempValidDrone.nextAvailableTime<<endl;
    validDrones.push_back(tempValidDrone);
    return true;
}

bool validDroneComparator(singleDrone &a,singleDrone &b){
    if(a.droneType == b.droneType) return a.availableTime>b.availableTime;
    else return a.droneType < b.droneType;
}

int main() {
    ios_base::sync_with_stdio(false);
    cin.tie(NULL),cout.tie(NULL);

    // constant values
    item tempItem = item("Item-1",1,5,8,5);
    items.push_back(tempItem);
    // items[0] = item("Item-1",1,5,8,5);
    tempItem = item("Item-2",6,5,10,8);
    items.push_back(tempItem);
    tempItem = item("Item-3",4,5,10,15);
    items.push_back(tempItem);
    tempItem =  item("Item-4",2,15,10,8);
    items.push_back(tempItem);
    tempItem = item("Item-5",5,20,15,10);
    items.push_back(tempItem);

    fstream file;
    file.open("Demand.csv",ios::in);
    string word,lines;
    vector<string>row;
    vector<chargingInfo>chargingInfos;
    
    // hardcode this data.
    chargingInfo tempChargingInfo = chargingInfo("WH1",INT32_MAX,5);
    chargingInfos.push_back(tempChargingInfo);
    tempChargingInfo = chargingInfo("WH2",INT32_MAX,5);
        chargingInfos.push_back(tempChargingInfo);
    tempChargingInfo = chargingInfo("WH3",INT32_MAX,5);
        chargingInfos.push_back(tempChargingInfo);
    tempChargingInfo = chargingInfo("A",1,3);
        chargingInfos.push_back(tempChargingInfo);
    tempChargingInfo = chargingInfo("B",1,3);
        chargingInfos.push_back(tempChargingInfo);
    tempChargingInfo = chargingInfo("C",5,3);
        chargingInfos.push_back(tempChargingInfo);
    tempChargingInfo = chargingInfo("D",1,3);
        chargingInfos.push_back(tempChargingInfo);
    tempChargingInfo = chargingInfo("E",4,3);
        chargingInfos.push_back(tempChargingInfo);

    vector<noFlyZone>noFlyZones;
        drone tempDrone = drone(0,2000,2,5,200,1,10,5);
        drones.push_back(tempDrone); 
        tempDrone = drone(1,2500,2.5,6,500,1,15,8);
        drones.push_back(tempDrone); 
        tempDrone = drone(2,3000,3,7,1000,2,20,13);
        drones.push_back(tempDrone); 
        tempDrone = drone(3,4000,3.5,8,2000,2,20,15);
        drones.push_back(tempDrone); 
        tempDrone = drone(4,5000,4,9,3000,2,30,20);
        drones.push_back(tempDrone); 
        tempDrone = drone(5,10000,5,10,5000,4,50,25);
        drones.push_back(tempDrone); 
    getline(file,lines);
    int cnt = 30;
    while(file && cnt>0){
        cnt--;
        row.clear();
        getline(file,lines);
        stringstream s(lines);
        while(getline(s,word,',')){
            row.push_back(word);
        }
        demand temp = demand(row);
        demands.push_back(temp);
        // for(auto x:demands){
        //     cout<<x.deliveryFailure<<" "<<x.x<<" "<<x.y<<" "<<x.z<<" "<<endl;
        // }
    }
    file.close();
    // input file 2;
    cout<<"chal ja bc"<<endl;
    file.open("Parameters.csv",ios::in);
    int maxSpeed;
    double chargingCost;
    int rownumber=0;
    getline(file,lines);
    while(file){
        rownumber++;
        row.clear();
        getline(file,lines);
        stringstream s(lines);
        while(getline(s,word,',')){
            row.push_back(word);
        }
        // for(auto x:row) cout<<x<<" ";
        // cout<<endl;
        if(rownumber==1){
            // cout<<"idhAR";
            maxSpeed = stoi(row[1]);
            for(auto &x:drones){
                x.maxSpeed = maxSpeed;
            }
        }
        else if(rownumber == 2){
            // cout<<"yaha bhi agya"<<endl;
            chargingCost = stod(row[1]);
        }
        else if(row[0][0] == 'X'){
            noFlyZone temp;
            temp.x[0] == stoi(row[1]);
            for(int i=1;i<=7;i++){
                row.clear();
                getline(file,lines);
                stringstream s(lines);
                while(getline(s,word,',')){
                    row.push_back(word);
                }
                temp.x[i] = stoi(row[1]); 
            }
            for(int i=0;i<8;i++){
                row.clear();
                getline(file,lines);
                stringstream s(lines);
                while(getline(s,word,',')){
                    row.push_back(word);
                }
                temp.y[i] = stoi(row[1]);
            }
            
            for(int i=0;i<8;i++){
                row.clear();
                getline(file,lines);
                stringstream s(lines);
                while(getline(s,word,',')){
                    row.push_back(word);
                }
                temp.z[i] = stoi(row[1]);
            }
            noFlyZones.push_back(temp);
        }
        else if(row[0][0] == 'W'){
            // cout<<"is it here?"<<endl;
            int counter = 0;
            chargingInfos[counter].xCoord = stoi(row[1]);
            for(int i=0;i<2;i++){
                row.clear();
                getline(file,lines);
                stringstream s(lines);
                while(getline(s,word,',')){
                    row.push_back(word);
                } 
                if(i%3==0){
                    chargingInfos[counter].yCoord = stoi(row[1]);
                } else if(i%3==1){
                    chargingInfos[counter].zCoord = stoi(row[1]);
                } else if(i%3==2){
                    counter++;
                    chargingInfos[counter].xCoord = stoi(row[1]);
                }
            }
        }
        else if(row[0] == "AX1"){
            // cout<<"should not be here?"<<endl;
            int counter = 3;
            chargingInfos[counter].xCoord = stoi(row[1]);
            for(int i=0;i<11;i++){
                row.clear();
                getline(file,lines);
                stringstream s(lines);
                while(getline(s,word,',')){
                    row.push_back(word);
                }
                if(i%2==0){
                    chargingInfos[counter].yCoord = stoi(row[1]);
                } else {
                    counter++;
                    chargingInfos[counter].xCoord  =stoi(row[1]);
                }
            }
        }
        else if(row[0] == "P1"){
            // cout<<"entering here"<<endl;
            int counter = 0;
            drones[counter].pq.p = stoi(row[1]);
            for(int i=0;i<5;i++){
                row.clear();
                getline(file,lines);
                stringstream s(lines);
                while(getline(s,word,',')){
                    row.push_back(word);
                }
                counter++;
                drones[counter].pq.p = stoi(row[1]);
            }
            counter=0;
            for(int i=0;i<6;i++){
                row.clear();
                getline(file,lines);
                stringstream s(lines);
                while(getline(s,word,',')){
                    row.push_back(word);
                }
                drones[counter].pq.q = stoi(row[1]);
                counter++;
            }
            counter=0;
            for(int i=0;i<6;i++){
                row.clear();
                getline(file,lines);
                stringstream s(lines);
                while(getline(s,word,',')){
                    row.push_back(word);
                }
                drones[counter].ec.a = stoi(row[1]);
                counter++;
            }

            counter=0;
            for(int i=0;i<6;i++){
                row.clear();
                getline(file,lines);
                stringstream s(lines);
                while(getline(s,word,',')){
                    row.push_back(word);
                }
                drones[counter].ec.b = stoi(row[1]);
                counter++;
            }

            counter=0;
            for(int i=0;i<6;i++){
                row.clear();
                getline(file,lines);
                stringstream s(lines);
                while(getline(s,word,',')){
                    row.push_back(word);
                }
                drones[counter].ec.c = stoi(row[1]);
                counter++;
            }

            counter=0;
            int indexCounter = 0;
            for(int i=0;i<6;i++){
                row.clear();
                getline(file,lines);
                stringstream s(lines);
                while(getline(s,word,',')){
                    row.push_back(word);
                }
                drones[counter].droneCount = stoi(row[1]);
                // cout<<"dronescounter "<<drones[counter].droneCount<<endl;
                for(int j=0;j<drones[counter].droneCount;j++){
                    singleDrone tempdrone = singleDrone(counter,0,drones[counter].batteryCapacity,indexCounter);
                    indexCounter++;
                    singleDrones.push_back(tempdrone);
                }
                counter++;
            }
        }
    }
    // implementation
    // cout<<"error is not here"<<endl;
    // cout<<"drones "<<drones.size()<<endl;
    // cout<<"singleDrones "<<singleDrones.size()<<endl;
    // cout<<"demands "<<demands.size()<<endl;
    int demandsMet=0;
    sort(demands.begin(),demands.end(),demandComparator);
    for(auto curDemand:demands){
        vector<singleDrone>validDrones;
        for(auto curDrone:singleDrones){
            checkOptimalDrone(curDemand,curDrone,validDrones);
        }
        sort(validDrones.begin(),validDrones.end(),validDroneComparator);
        // cout<<"size "<<validDrones.size()<<endl;
        if(validDrones.size() > 0){
            demandsMet++;
            singleDrones[validDrones[0].index].availableTime = validDrones[0].nextAvailableTime;
            cout<<validDrones[0].nextAvailableTime<<" "<<validDrones[0].index<<endl<<endl;
            // singleDrones[validDrones[0].index].remainingBattery = 2000; 
            // drones[singleDrones[validDrones[0].index].droneType].batteryCapacity;
        }

    }
    int sz = demands.size();
    cout<<double(demandsMet)/sz<<endl;
        


    // vector<energyConsumption> energyConsumptions;
    // vector<speedProfile>speedProfiles;





    // // output file
    // map<string,vector<dronePathOutput>> DronePathOutputs;
    // vector<droneCostOutput>DroneCostOutputs;

}