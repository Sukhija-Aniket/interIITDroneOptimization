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
    double a,b,c;
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
    double availableTime = 0;
    int remainingBattery;
    double nextAvailableTime=0;
    int endTime = 0;
    int lastDemandId=0;
    double rechargeTime=0, restingTime=0,maintainanceCost=0, energyCost=0;

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
    double x,y,z;
    string activity;
    int speed;
    double batteryConsumption;
    double EnergyCost;
    int totalWeight = 0;
};

struct noFlyZone{
    int x[8];
    int y[8];
    int z[8];
};

struct outputPathRow{
    int demandId,time,totalWeight = 0;
    double x,y,z,speed, batteryConsumed, energyCost;
    string droneId;
    string day,activity;
    void printParams(){
        cout<<"  DemandId is "<<demandId;
        cout<<endl;
        cout<<"  Time is "<<time;
        cout<<endl;
        cout<<"  Speed is "<<speed;
        cout<<endl;
        cout<<"  TotalWeight is "<<totalWeight;
        cout<<endl;
        cout<<"  X is "<<x;
        cout<<endl;
        cout<<"  Y is "<<y;
        cout<<endl;
        cout<<"  Z is "<<z;
        cout<<endl;
        cout<<"  batteryConsumed is "<<batteryConsumed;
        cout<<endl;
        cout<<"  energyCost is "<<energyCost;
        cout<<endl;
        cout<<"  droneId is "<<droneId;
        cout<<endl;
        cout<<"  day is "<<day;
        cout<<endl;
        cout<<"  Activity is "<<activity;
        cout<<endl;
    }
};
struct outputCostRow{
    string droneId;
    string day;
    double restingTime, chargingTime, maintainanceCost, energyCost;
    void printParams(){
        cout<<"  droneId is "<<droneId;
        cout<<endl;
        cout<<"  day is "<<day;
        cout<<endl;
        cout<<"  restingTime is "<<restingTime;
        cout<<endl;
        cout<<"  chargingTime is "<<chargingTime;
        cout<<endl;
        cout<<"  maintainanceCost is "<<maintainanceCost;
        cout<<endl;
        cout<<"  energyCost is "<<energyCost;
        cout<<endl;
    }
};

// global variables
vector<item>items;
vector<demand>demands;
vector<singleDrone> singleDrones;
vector<drone>drones;
vector<outputPathRow> outputPath;
vector<outputCostRow> outputCost;
double chargingCost;

// utility functions
int getSeconds(string time){
    int counter = 0;
    string hours="",minutes="",seconds="";
    for(auto x:time){
        if(x==':'){
            counter++;
        } else if(counter == 0){
            hours += x;
        } else if(counter == 1){
            minutes += x;
        } else if(counter == 2){
            seconds += x;
        }
    }
    int Seconds = (stoi(hours)-8) * 3600 + stoi(minutes)*60 + stoi(seconds);
    return Seconds;
}
map<string, int> demandIdmapping;
string convertToString(int x){
    int y = x;
    x++;
    int counter = 0;
    for(int i=0;i<6;i++){
        if(x>drones[i].droneCount){
            x -= drones[i].droneCount;
        } else {
            string s="";
            s += "D" + to_string(i+1);
            s += char(x+64);
            demandIdmapping[s] = y;
            return s;
        }
    }
    return "error";
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
    // cout<<curDemand.demandId<<": "<<curDemand.deliveryEndTime<<" "<<curDrone.availableTime + timeUsed + 360<<endl;
    if(curDrone.availableTime + timeUsed + 360 > curDemand.deliveryEndTime) return false;
    double extraTime = curDrone.availableTime + timeUsed + 180;
    // cout<<" this line "<<netWeight<<" "<<tempDrone.ec.a<<" "<<tempDrone.ec.b<<" "<<netSpeedXY<<" "<<distanceXY<<endl;
    batteryUsage += netWeight * (tempDrone.ec.a + tempDrone.ec.b* (netSpeedXY)) * (distanceXY/netSpeedXY);
    batteryUsage += netWeight * (tempDrone.ec.a + tempDrone.ec.c*netSpeedZUp) * (distanceZ/netSpeedZUp);
    batteryUsage += tempDrone.baseWeight * (tempDrone.ec.a + tempDrone.ec.b* (netSpeedReturn)) * (distanceXY/netSpeedReturn);
    batteryUsage += tempDrone.baseWeight * (tempDrone.ec.a + tempDrone.ec.c* (netSpeedReturn)) * (distanceZ/netSpeedReturn);
    
    
    // cout<<curDrone.remainingBattery<<" battery "<<batteryUsage<<endl;
    if(batteryUsage > curDrone.remainingBattery) return false;
    timeUsed += 360 + (distanceXY + distanceZ)/netSpeedReturn;
    // cout<<"time "<<timeUsed<<endl;
    // timeUsed += max(0.0,double((2000-(curDrone.remainingBattery - batteryUsage))*36)/50LL);
    timeUsed += double((tempDrone.batteryCapacity -(curDrone.remainingBattery-batteryUsage))*36)/50;
    // cout<<"time used "<<double((tempDrone.batteryCapacity -(curDrone.remainingBattery-batteryUsage))*36)/50<<endl;
    
    singleDrone tempValidDrone = curDrone;

    timeUsed += max(0.0,curDemand.deliveryStartTime - extraTime);
    tempValidDrone.nextAvailableTime = timeUsed + curDrone.availableTime;

    // cout<<"assigning: "<<timeUsed<<" "<<curDrone.availableTime<<endl;
    // cout<<timeUsed<<" "<<curDrone.availableTime<<" "<<tempValidDrone.nextAvailableTime<<endl;
    validDrones.push_back(tempValidDrone);
    return true;
}

void AssignOutput(outputPathRow &outputRow, int demandId, int droneId, string day,int time, double x,double y, double z, string activity, double speed, double batteryConsumed, double energyCost, int weight){
    outputRow.demandId = demandId;
    outputRow.droneId = convertToString(droneId);
    outputRow.day = day;
    outputRow.time = time;
    outputRow.x = x;
    outputRow.y = y;
    outputRow.z = z;
    outputRow.activity = activity;
    outputRow.speed = speed;
    outputRow.batteryConsumed = batteryConsumed;
    outputRow.energyCost = energyCost;
    outputRow.totalWeight = weight;
}

void AssignOutputCost(outputCostRow &outputRow, int droneId, string day, double restingTime,double chargingTime,double maintainanceCost,double energyCost){
    outputRow.droneId = droneId;
    outputRow.day = day;
    outputRow.restingTime = restingTime;
    outputRow.chargingTime = chargingTime;
    outputRow.maintainanceCost = maintainanceCost;
    outputRow.energyCost = energyCost;
}

void changeMapParams(unordered_map<string, outputCostRow> &outputCostMap,vector<string> row){
    if(row.size()==0)
        return;
    outputCostMap[row[1]].droneId = row[1];
    outputCostMap[row[1]].restingTime += row[7][0]=='R';
    outputCostMap[row[1]].chargingTime += row[7][0]=='C';
    outputCostMap[row[1]].day = "Day 1";

    unordered_map<int, pair<double, double>> mapper;
    mapper[1] = {10,5/3600.0};
    mapper[2] = {15,8/3600.0};
    mapper[3] = {20,13/3600.0};
    mapper[4] = {20,15/3600.0};
    mapper[5] = {30,20/3600.0};
    mapper[6] = {50,25/3600.0};

    outputCostMap[row[1]].maintainanceCost += ((row[7][0] == 'T')?(mapper[drones[stoi(row[1])].droneType+1].second):0);
    outputCostMap[row[1]].energyCost += stod(row[10]);
}

void OptimalDroneParamsPusher(demand &curDemand,singleDrone &curDrone){
    drone tempDrone = drones[curDrone.droneType];
    item tempItem = items[curDemand.itemId];
    int netWeight = tempItem.weight + tempDrone.baseWeight;
    double netSpeedXY = tempDrone.maxSpeed - tempDrone.pq.p* double(tempItem.weight)/double(tempDrone.payloadCapacity);
    double netSpeedZUp = tempDrone.maxSpeed - tempDrone.pq.q * double(tempItem.weight)/double(tempDrone.payloadCapacity);
    double netSpeedReturn = tempDrone.maxSpeed;
    double batteryUsage = 0;
    double distanceXY = sqrt(curDemand.x*curDemand.x + curDemand.y*curDemand.y);
    double distanceZ = curDemand.z;
    double timeUsed = distanceXY/netSpeedXY + distanceZ/netSpeedZUp;
    double extraTime = curDrone.availableTime + timeUsed + 180;
    batteryUsage += netWeight * (tempDrone.ec.a + tempDrone.ec.b* (netSpeedXY)) * (distanceXY/netSpeedXY);
    batteryUsage += netWeight * (tempDrone.ec.a + tempDrone.ec.c*netSpeedZUp) * (distanceZ/netSpeedZUp);
    batteryUsage += tempDrone.baseWeight * (tempDrone.ec.a + tempDrone.ec.b* (netSpeedReturn)) * (distanceXY/netSpeedReturn);
    batteryUsage += tempDrone.baseWeight * (tempDrone.ec.a + tempDrone.ec.c* (netSpeedReturn)) * (distanceZ/netSpeedReturn);
    timeUsed += 360 + (distanceXY + distanceZ)/netSpeedReturn;
    timeUsed += double((tempDrone.batteryCapacity -(curDrone.remainingBattery-batteryUsage))*36)/50;
    singleDrone tempValidDrone = curDrone;
    timeUsed += max(0.0,curDemand.deliveryStartTime - extraTime);

    double droneStartTime = curDrone.availableTime;
    tempValidDrone.nextAvailableTime = timeUsed + curDrone.availableTime;
    double droneEndTime = tempValidDrone.nextAvailableTime;

    int demandId = curDemand.demandId;
    int droneId = curDrone.index;
    int droneTypeId = curDrone.droneType;
    int warehouseResting = 180;
    netSpeedXY = netSpeedXY;
    double xyTravel = distanceXY/netSpeedXY;
    double zTravel = distanceZ/netSpeedZUp;
    double possibleRestingTimeAtDrop = max(0.0,curDemand.deliveryStartTime - extraTime);
    int dropingTime = 180;
    double zReturn = distanceZ/netSpeedReturn;
    double xyReturn = distanceXY/netSpeedReturn;
    double recharge = double((tempDrone.batteryCapacity -(curDrone.remainingBattery-batteryUsage))*36)/50;

    curDrone.endTime = max(int(droneEndTime),curDrone.endTime);
    // cout<<"updating endTime "<<curDrone.endTime<<endl;
    curDrone.lastDemandId = curDemand.demandId;
    for(int i = ceil(droneStartTime);i<=droneEndTime;i++){
        outputPathRow outputRow;
        double iamx,iamy,iamz;
        if(i<=droneStartTime+possibleRestingTimeAtDrop){
            iamx = 0;
            iamy = 0;
            iamz = 0;
            AssignOutput(outputRow, demandId, droneId, "Day 1",i, iamx, iamy,iamz, "R-WH1", 0, 0, 0, drones[droneTypeId].baseWeight);
        }
        else if(i<=droneStartTime+warehouseResting+possibleRestingTimeAtDrop){
            iamx = 0;
            iamy = 0;
            iamz = 0;
            AssignOutput(outputRow, demandId, droneId, "Day 1",i, iamx, iamy,iamz, "PU-WH1", 0, 0, 0, drones[droneTypeId].baseWeight);
        }
        else if(i<=droneStartTime+warehouseResting+xyTravel + possibleRestingTimeAtDrop){
            iamx = (i - (droneStartTime+warehouseResting+possibleRestingTimeAtDrop))*netSpeedXY*curDemand.x/distanceXY;
            iamy = (i - (droneStartTime+warehouseResting+possibleRestingTimeAtDrop))*netSpeedXY*curDemand.y/distanceXY;
            iamz = 0;
            double iamxyPower = netWeight * (tempDrone.ec.a + tempDrone.ec.b* (netSpeedXY));
            double iamxyEnergy = iamxyPower * (distanceXY/netSpeedXY)*chargingCost;
            AssignOutput(outputRow, demandId, droneId, "Day 1",i, iamx, iamy, iamz, "T-L", netSpeedXY, iamxyPower, iamxyEnergy, drones[droneTypeId].baseWeight+items[curDemand.itemId].weight);
        }
        else if(i - (droneStartTime + warehouseResting + xyTravel+possibleRestingTimeAtDrop)<1.0){
            // cout<<"this is the value "<<(i - (droneStartTime + warehouseResting + xyTravel))<<endl;
            // cout<<"here as well"<<endl;
            iamx = double(curDemand.x);
            // cout<<curDemand.x<<endl;
            iamy = double(curDemand.y);
            iamz = 0;
            double iamxyPower = netWeight * (tempDrone.ec.a + tempDrone.ec.b* (netSpeedXY));
            double iamxyEnergy = iamxyPower * (distanceXY/netSpeedXY)*chargingCost;
            AssignOutput(outputRow,demandId,droneId,"Day 1",i,iamx,iamy,iamz,"T-L",netSpeedXY,iamxyPower,iamxyEnergy,drones[droneTypeId].baseWeight + items[curDemand.itemId].weight);
        }
        else if(i<=droneStartTime+warehouseResting+xyTravel+zTravel+possibleRestingTimeAtDrop){
            iamz = (int(i - (droneStartTime+warehouseResting+xyTravel+possibleRestingTimeAtDrop)))*netSpeedZUp;
            double iamzPower = netWeight * (tempDrone.ec.a + tempDrone.ec.c * (netSpeedZUp));
            double iamzEnergy = iamzPower * chargingCost;
            AssignOutput(outputRow, demandId, droneId, "Day 1",i, iamx, iamy, iamz, "T-L", netSpeedZUp, iamzPower, iamzEnergy, drones[droneTypeId].baseWeight+items[curDemand.itemId].weight);
        }

        else if(i - (droneStartTime+warehouseResting+xyTravel+zTravel+possibleRestingTimeAtDrop) < 1.0){
            // cout<<"entering here"<<endl;
            iamz = curDemand.z;
            double iamzPower = netWeight * (tempDrone.ec.a + tempDrone.ec.c * (netSpeedZUp));
            double iamzEnergy = iamzPower * chargingCost;
            AssignOutput(outputRow, demandId, droneId, "Day 1",i, iamx, iamy, iamz, "T-L", netSpeedZUp, iamzPower, iamzEnergy, drones[droneTypeId].baseWeight+items[curDemand.itemId].weight);
        }
        
        else if(i<=droneStartTime+warehouseResting+xyTravel+zTravel+possibleRestingTimeAtDrop+dropingTime){
            iamx = int(iamx);
            iamy = int(iamy);
            iamz = int(iamz);
            AssignOutput(outputRow, demandId, droneId, "Day 1",i, iamx, iamy, iamz, "D"+to_string(curDemand.demandId), 0, 0, 0, drones[droneTypeId].baseWeight+items[curDemand.itemId].weight);
        }
        else if(i<=droneStartTime+warehouseResting+xyTravel+zTravel+possibleRestingTimeAtDrop+dropingTime+zReturn){
            iamz = curDemand.z - (i - (droneStartTime+warehouseResting+xyTravel+zTravel+possibleRestingTimeAtDrop+dropingTime))*netSpeedReturn;
            double iamzPower = tempDrone.baseWeight * (tempDrone.ec.a + tempDrone.ec.c * (netSpeedReturn));
            double iamzEnergy = iamzPower * chargingCost;
            AssignOutput(outputRow, demandId, droneId, "Day 1",i, iamx, iamy, iamz, "T-E", netSpeedReturn, iamzPower, iamzEnergy, drones[droneTypeId].baseWeight);
        }
        else if(i -(droneStartTime+warehouseResting+xyTravel+zTravel+possibleRestingTimeAtDrop+dropingTime+zReturn)<1.0){
            iamz = 0;
            double iamzPower = tempDrone.baseWeight * (tempDrone.ec.a + tempDrone.ec.c * (netSpeedReturn));
            double iamzEnergy = iamzPower * chargingCost;
            AssignOutput(outputRow, demandId, droneId, "Day 1",i, iamx, iamy, iamz, "T-E", netSpeedReturn, iamzPower, iamzEnergy, drones[droneTypeId].baseWeight);
        }
        else if(i<=droneStartTime+warehouseResting+xyTravel+zTravel+possibleRestingTimeAtDrop+dropingTime+zReturn+xyReturn){
            iamx = curDemand.x - (int(i - (droneStartTime+warehouseResting+xyTravel+zTravel+possibleRestingTimeAtDrop+dropingTime+zReturn)))*netSpeedReturn*curDemand.x/distanceXY;
            iamy = curDemand.y - (int(i - (droneStartTime+warehouseResting+xyTravel+zTravel+possibleRestingTimeAtDrop+dropingTime+zReturn)))*netSpeedReturn*curDemand.y/distanceXY;
            iamz = 0;
            double iamxyPower = tempDrone.baseWeight * (tempDrone.ec.a + tempDrone.ec.b* (netSpeedReturn));
            double iamxyEnergy = iamxyPower * (distanceXY/netSpeedReturn)*chargingCost;
            AssignOutput(outputRow, demandId, droneId, "Day 1",i, iamx, iamy, iamz, "T-E", netSpeedReturn, iamxyPower, iamxyEnergy, drones[droneTypeId].baseWeight);
        }
        else if(i-(droneStartTime+warehouseResting+xyTravel+zTravel+possibleRestingTimeAtDrop+dropingTime+zReturn+xyReturn)<1.0){
            if(curDemand.x >= 0){
                iamx = max(0.0,curDemand.x - (int(i - (droneStartTime+warehouseResting+xyTravel+zTravel+possibleRestingTimeAtDrop+dropingTime+zReturn)))*netSpeedReturn*curDemand.x/distanceXY);
            } else {
                iamx = min(0.0,curDemand.x - (int(i - (droneStartTime+warehouseResting+xyTravel+zTravel+possibleRestingTimeAtDrop+dropingTime+zReturn)))*netSpeedReturn*curDemand.x/distanceXY);
            }
            if(curDemand.y>=0){
                iamy = max(0.0,curDemand.y - (int(i - (droneStartTime+warehouseResting+xyTravel+zTravel+possibleRestingTimeAtDrop+dropingTime+zReturn)))*netSpeedReturn*curDemand.y/distanceXY);
            } else {
                iamy = min(0.0,curDemand.y - (int(i - (droneStartTime+warehouseResting+xyTravel+zTravel+possibleRestingTimeAtDrop+dropingTime+zReturn)))*netSpeedReturn*curDemand.y/distanceXY);
            }
            double iamxyPower = tempDrone.baseWeight * (tempDrone.ec.a + tempDrone.ec.b* (netSpeedReturn));
            double iamxyEnergy = iamxyPower * (distanceXY/netSpeedReturn)*chargingCost;
            AssignOutput(outputRow, demandId, droneId, "Day 1",i, iamx, iamy, iamz, "T-E", netSpeedReturn, iamxyPower, iamxyEnergy, drones[droneTypeId].baseWeight);
        }
        else if(i<=droneStartTime+warehouseResting+xyTravel+zTravel+possibleRestingTimeAtDrop+dropingTime+zReturn+xyReturn+recharge){
            AssignOutput(outputRow, demandId, droneId, "Day 1",i, 0, 0, 0, "C-WH1", 0, 0, 0, drones[droneTypeId].baseWeight);
        }
        // else{
        //     cout<<"MAdarchod case\n";
        // }
        // cout<<"Params of row are-:\n";
        // outputRow.printParams();
        // cout<<endl;
        outputPath.push_back(outputRow);
        // int x = outputPath.size();
        // cout<<"Params of row are-:\n";
        // outputPath[x-1].printParams();
        // cout<<endl;
        // cout<<"-----------------\n";
    }
}

bool validDroneComparator(singleDrone &a,singleDrone &b){
    if(a.droneType == b.droneType) return a.availableTime>b.availableTime;
    else return a.droneType < b.droneType;
}

bool outputPathComparator(outputPathRow &a,outputPathRow &b){
    if(a.droneId != b.droneId) return a.droneId<b.droneId;
    else return a.time < b.time;
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
        if(row[0]!="WH1") continue;
        demands.push_back(temp);
        // for(auto x:demands){
        //     cout<<x.deliveryFailure<<" "<<x.x<<" "<<x.y<<" "<<x.z<<" "<<endl;
        // }
    }
    file.close();
    // input file 2;
    // cout<<"chal ja bc"<<endl;
    file.open("Parameters.csv",ios::in);
    int maxSpeed;
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
            for(int i=0;i<3;i++){
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
                drones[counter].ec.a = stod(row[1]);
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

                drones[counter].ec.b = stod(row[1]);
                // cout<<"other line "<<drones[counter].ec.b<<" "<<stod(row[1])<<endl;
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
                drones[counter].ec.c = stod(row[1]);
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
    vector<int>tempDemandId;
    sort(demands.begin(),demands.end(),demandComparator);
    for(auto curDemand:demands){
        bool checker = false;
        vector<singleDrone>validDrones;
        for(auto curDrone:singleDrones){
            checker |= checkOptimalDrone(curDemand,curDrone,validDrones);
        }
        if(checker == false) {
            tempDemandId.push_back(curDemand.demandId);
        }
        sort(validDrones.begin(),validDrones.end(),validDroneComparator);
        // cout<<"size "<<validDrones.size()<<endl;
        if(validDrones.size() > 0){
            demandsMet++;
            OptimalDroneParamsPusher(curDemand, validDrones[0]);
            singleDrones[validDrones[0].index].availableTime = validDrones[0].nextAvailableTime;
            singleDrones[validDrones[0].index].endTime = validDrones[0].endTime;
            singleDrones[validDrones[0].index].lastDemandId = validDrones[0].lastDemandId;
            // cout<<validDrones[0].nextAvailableTime<<" "<<validDrones[0].index<<endl<<endl;
            // singleDrones[validDrones[0].index].remainingBattery = 2000; 
            // drones[singleDrones[validDrones[0].index].droneType].batteryCapacity;
        }

    }
    for(auto drone:singleDrones){
        // if(drone.endTime == 0) continue;
        if(drone.endTime>=14400){
            // cout<<drone.index<<" "<<drone.endTime<<endl;
            outputPathRow outputRow;
            AssignOutput(outputRow, drone.lastDemandId, drone.index, "Day 1",drone.endTime+1, 0, 0, 0, "End", 0, 0, 0, drones[drone.index].baseWeight);
            outputPath.push_back(outputRow);
            // outputPath.back().printParams();
            // cout<<endl;
        }
        else {
            for(int i=drone.endTime+1;i<=14400;i++){
                outputPathRow outputRow;
                AssignOutput(outputRow,drone.lastDemandId,drone.index,"Day 1",i, 0, 0, 0, "R-WH1", 0, 0, 0, drones[drone.droneType].baseWeight);
                outputPath.push_back(outputRow);
            }
            // cout<<drone.lastDemandId<<endl;
            outputPathRow outputRow;
            AssignOutput(outputRow, drone.lastDemandId, drone.index, "Day 1",14401, 0, 0, 0, "End", 0, 0, 0, drones[drone.droneType].baseWeight);
            outputPath.push_back(outputRow);
        }

    }
    // for(auto x:tempDemandId){
    //     cout<<"id is :"<<x<<endl;
    // }
    // int sz = demands.size();
    // cout<<double(demandsMet)/sz<<endl;
    
    sort(outputPath.begin(), outputPath.end(), outputPathComparator);
    ofstream myfile;
    myfile.open ("outputPath_scenario3_day3.csv");
    myfile<<"demandId,droneId,day,time,x,y,z,activity,speed,batteryConsumed,energyCost,totalWeight\n";
    
    for(auto row:outputPath){
        myfile<<row.demandId<<",";
        myfile<<row.droneId<<",";
        myfile<<row.day<<",";
        myfile<<row.time<<",";
        myfile<<row.x<<",";
        myfile<<row.y<<",";
        myfile<<row.z<<",";
        myfile<<row.activity<<",";
        myfile<<row.speed<<",";
        myfile<<row.batteryConsumed<<",";
        myfile<<row.energyCost<<",";
        myfile<<row.totalWeight;
        myfile<<endl;
    }
    myfile.close();
    // vector<energyConsumption> energyConsumptions;
    // vector<speedProfile>speedProfiles;

    {
    fstream file;
    file.open("outputPath.csv",ios::in);
    string word,lines;
    unordered_map<string,outputCostRow> outputCostMap;
    getline(file,lines);
    while(file){
        row.clear();
        getline(file,lines);
        stringstream s(lines);
        while(getline(s,word,',')){
            row.push_back(word);
        }
        changeMapParams(outputCostMap, row);
    }
    // for(auto drone:singleDrones){
    //     outputCostMap[drone.index].maintainanceCost += drone.maintainanceCost;
    // }
    ofstream myfile;
    myfile.open ("outputCost.csv");
    myfile<<"DroneID, Day, RestingTime (s), Charging time (s), Maintainance Cost ($), Energy Cost ($)\n";
    for(auto row:outputCostMap){
        myfile<<row.first<<",";
        myfile<<"Day 1"<<",";
        myfile<<row.second.restingTime<<",";
        myfile<<row.second.chargingTime<<",";
        myfile<<row.second.maintainanceCost<<",";
        myfile<<row.second.energyCost;
        myfile<<endl;
    }
    myfile.close();
    file.close();
    }

    // // output file
    // map<string,vector<dronePathOutput>> DronePathOutputs;
    // vector<droneCostOutput>DroneCostOutputs;
    return 0;
}