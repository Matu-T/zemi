#include <iostream>
#include <algorithm>
#include <vector>
#include <map>
#include <string>
#include <cmath>
#include <fstream>
#include <sstream>	
#include "Eigen/core"
#include "Eigen/LU"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using namespace std;

//自動車,二輪車 基底構造体
struct Vehicle
{
protected:
    double sx_;//出発地x座標
    double sy_;//出発地y座標
    double gx_;//目的地x座標
    double gy_;//目的地y座標
    double dis_;//距離
    double park_;//駐車代 
    double fuelpk_;//キロ当たり燃費
    double fuel_;//燃費
    double speed_;//速度

public:
    Vehicle(double sx, double sy, double gx, double gy, double p, double fp, double sp)
     : sx_(sx), sy_(sy), gx_(gx), gy_(gy), dis_(0), park_(p), fuelpk_(fp), fuel_(0), speed_(sp) {}

    //仮想関数 派生クラスでoverride
    virtual double getDistance() = 0;

    double getCost(){
        fuel_ = getDistance() * fuelpk_;
        return fuel_ + park_;
    }

    double getTime(){
        return getDistance() /speed_;
    }

};

//自動車(送迎・ライドシェア含む)
struct Car : Vehicle
{
public:
    Car(double sx, double sy, double gx, double gy, double p, double fp, double sp)
        : Vehicle(sx, sy, gx, gy, p, fp, sp) {}

    double getDistance() override{
        double sl = sqrt(pow(gx_ - sx_, 2.0) +  pow(gy_ - sy_, 2.0));
        //ここでは直線距離の1.2倍を実際の距離とする
        dis_ = sl * 1.2;
        return dis_;
    }
};


//バイク
struct Motor : Vehicle
{
public:
    Motor(double sx, double sy, double gx, double gy, double p, double fp, double sp)
        : Vehicle(sx, sy, gx, gy, p, fp, sp) {}

    double getDistance() override{
        double sl = sqrt(pow(gx_ - sx_, 2.0) +  pow(gy_ - sy_, 2.0));
        //ここでは直線距離の1.1倍を実際の距離とする
        dis_ = sl * 1.1;
        return dis_;
    }
};

//自転車
struct Bicycle : Vehicle
{
public:
    Bicycle(double sx, double sy, double gx, double gy, double p, double fp, double sp)
        : Vehicle(sx, sy, gx, gy, p, fp, sp) {}

    double getDistance() override{
        double sl = sqrt(pow(gx_ - sx_, 2.0) +  pow(gy_ - sy_, 2.0));
        //ここでは直線距離の1.05倍を実際の距離とする
        dis_ = sl * 1.05;
        return dis_;
    }
};


//公共交通機関 基底構造体
struct Transp
{
protected:
    double sx_;//出発地x座標
    double sy_;//出発地y座標
    double gx_;//目的地x座標
    double gy_;//目的地y座標
    double dis_;//距離
    map<double, double> farebd_;//std::map farebd_(fare by distance)は、料金が変わる距離とその金額を格納fare[0] = 100 fare[0.255] = 270,...
    double fare_;
    double speed_;//平均速度
public:
    Transp(double sx, double sy, double gx, double gy, map<double, double>& farebd, double speed)
     : sx_(sx), sy_(sy), gx_(gx), gy_(gy), dis_(0), farebd_(farebd), fare_(0), speed_(speed) {}

    virtual double getDistance() = 0;

    double getCost(){
        double distance = getDistance();

        //mapの各farebd_の要素について
        for(const auto& i : farebd_){
            //farebd_はソートされている必要がある
            fare_ = i.second;
            if(distance < i.first){
                //「次の上がり幅」よりも小さかったら、その時の運賃を出力
                return fare_;
            }
        }
        //最高値
        return fare_;
    }

    //通過待ち時間などは考えない
    double getTime(){
        double distance = getDistance();
        return distance / speed_;
    }

};

//鉄道(LRTも含む)
struct Train : Transp
{
public:
    Train(double sx, double sy, double gx, double gy, map<double, double>& farebd, double speed)
        : Transp(sx,sy,gx,gy,farebd,speed) {}

    //電車のみ バスは異なる
    double getDistance() override{
        double sl = sqrt(pow(gx_ - sx_, 2.0) +  pow(gy_ - sy_, 2.0));
        //ここでは直線距離の1.25倍を実際の距離とする
        dis_ = sl * 1.25;
        return dis_;
    }
};

//バス
struct Bus : Transp
{
public:
    Bus(double sx, double sy, double gx, double gy, map<double, double>& farebd, double speed)
        : Transp(sx,sy,gx,gy,farebd,speed) {}

    double getDistance() override{
        double sl = sqrt(pow(gx_ - sx_, 2.0) +  pow(gy_ - sy_, 2.0));
        //ここでは直線距離の1.1倍を実際の距離とする
        dis_ = sl * 1.1;
        return dis_;
    }
};

//バス
struct Autobus : Transp
{
public:
    Autobus(double sx, double sy, double gx, double gy, map<double, double>& farebd, double speed)
        : Transp(sx,sy,gx,gy,farebd,speed) {}

    double getDistance() override{
        double sl = sqrt(pow(gx_ - sx_, 2.0) +  pow(gy_ - sy_, 2.0));
        //ここでは直線距離の1.2倍を実際の距離とする
        dis_ = sl * 1.2;
        return dis_;
    }
};

//タクシー
struct Taxi
{
protected:
    double sx_;//出発地x座標
    double sy_;//出発地y座標
    double gx_;//目的地x座標
    double gy_;//目的地y座標
    double dis_;//距離
    map<double, double> farebd_;
    double fare_;
    double speed_;
public:
    Taxi(double sx, double sy, double gx, double gy, map<double, double>& farebd, double speed)
     : sx_(sx), sy_(sy), gx_(gx), gy_(gy), dis_(0), farebd_(farebd), fare_(0), speed_(speed) {}

    double getDistance(){
        double sl = sqrt(pow(gx_ - sx_, 2.0) +  pow(gy_ - sy_, 2.0));
        //ここでは直線距離の1.2倍を実際の距離とする
        dis_ = sl * 1.2;
        return dis_;
    }

    double getCost(){
        double distance = getDistance();

        //mapの各farebd_の要素について
        for(const auto& i : farebd_){
            fare_ = i.second;
            if(distance < i.first){
                //「次の上がり幅」よりも小さかったら、その時の運賃を出力
                return fare_;
            }
        }
        //最高値
        return fare_;
    }

    double getTime(){
        return getDistance() /speed_;
    }
};

//徒歩
struct Walk
{
protected:
    double sx_;//出発地x座標
    double sy_;//出発地y座標
    double gx_;//目的地x座標
    double gy_;//目的地y座標
    double dis_;//距離
    double speed_;//速度

public:
    Walk(double sx, double sy, double gx, double gy, double sp)
     : sx_(sx), sy_(sy), gx_(gx), gy_(gy), dis_(0), speed_(sp) {}

    double getDistance(){
        double sl = sqrt(pow(gx_ - sx_, 2.0) +  pow(gy_ - sy_, 2.0));
        //ここでは直線距離の1.05倍を実際の距離とする
        dis_ = sl * 1.05;
        return dis_;
    }

    double getTime(){
        return getDistance() /speed_;
    }
};

struct DataSet {
    vector<vector<double>> revealedX;
    vector<vector<double>> revealedY;
    vector<vector<int>> revealedMeans;
    vector<vector<double>> statedX;
    vector<vector<double>> statedY;
    vector<vector<int>> statedMeans;
    vector<vector<double>> alternateX;
    vector<vector<double>> alternateY;
    vector<vector<int>> alternateMeans;
};

template<typename T>
T ConvertValue(const string& value){
    try {
        if constexpr (is_same_v<T, int>){// int型であれば変換
            return stoi(value);
        } else if constexpr (is_same_v<T, double>){
            return stod(value);
        }
    } catch (const exception& e) {
        //cerr << "変換エラー: " << value << endl;
    }
    return T{};
}

//2次元配列で格納
template<typename T>
void LoadFile (string fileName, string type, vector<vector<T>>& vec){
    ifstream file(fileName); // CSVファイルを開く
    string line; // 行を格納する変数

    // ファイルが開けたか確認
    if (file.is_open()) {
        while (getline(file, line)) { // 行を1行ずつ読み込む
            stringstream ss(line); // 行をストリームに変換
            string value;
            vector<T> row; // 行のデータを格納

            // カンマで区切られた値を読み込む
            while (getline(ss, value, ',')) {
                try {
                    if(type == "double"){
                        row.emplace_back(stod(value)); // 値を double に変換して行に追加
                    } else if(type == "int"){
                        row.emplace_back(stoi(value)); // 値を int に変換して行に追加
                    }
                } catch (const invalid_argument& e) {
                    //cerr << "変換エラー: " << value << endl;
                } catch (const out_of_range& e) {
                    cerr << "値が範囲外です: " << value << endl;
                }
            }
            vec.emplace_back(row); // 行をデータに追加
        }
        file.close(); // ファイルを閉じる
    } else {
        std::cerr << "ファイルを開けませんでした。" << std::endl; // エラーメッセージ
    }

}

void LoadFileAsFare (string fileName, map<double, double>& m){
    ifstream file(fileName); // CSVファイルを開く
    string line; // 行を格納する変数

    // ファイルが開けたか確認
    if (file.is_open()) {
        while (getline(file, line)) { // 行を1行ずつ読み込む
            stringstream ss(line); // 行をストリームに変換
            string distance;
            string cost;

            getline(ss, distance, ',');
            getline(ss, cost, ',');
            try{
                m[stod(distance)] = stod(cost);
            } catch (const invalid_argument& e) {
                //cerr << "変換エラー: " << distance << endl;
                //cerr << "変換エラー: " << cost<< endl;
            } catch (const out_of_range& e) {
                cerr << "範囲外です: " << distance << endl;
                cerr << "範囲外です: " << cost<< endl;
            }
        }
        file.close(); // ファイルを閉じる
    } else {
        std::cerr << "ファイルを開けませんでした。" << std::endl; // エラーメッセージ
    }
}

void LoadFileAsMap (string fileName, string type, vector<map<string, double>>& m){
    ifstream file(fileName); // CSVファイルを開く
    string line; // 行を格納する変数

    // ファイルが開けたか確認
    if (file.is_open()) {
        vector<string> keys; // keyを格納

        //key(1行目)の読み込み
        if(getline(file, line)){
            stringstream ss(line);
            string key;
            while(getline(ss, key, ',')){
                keys.emplace_back(key);
            }
        }

        int cnt = 0;// 個人を表すインデックス
        while (getline(file, line)) { // 残りの行を1行ずつ読み込む
            stringstream ss(line); // 行をストリームに変換
            string value;
            int index = 0;

            m.emplace_back(map<string, double>());// mapの追加 

            // カンマで区切られた値を読み込む
            while (getline(ss, value, ',')) {
                try {
                    if(type == "double"){
                        m[cnt][keys[index]] = stod(value); // 値を double に変換して行に追加
                    } else if(type == "int"){
                        m[cnt][keys[index]] = stoi(value); // 値を int に変換して行に追加
                    }
                } catch (const invalid_argument& e) {
                    //cerr << "変換エラー: " << value << endl;
                } catch (const out_of_range& e) {
                    cerr << "値が範囲外です: " << value << endl;
                }
                //カンマ毎に別のkeyへ格納
                index ++;
            }
            // 1行終わるごとに個人+1
            cnt ++;
            }
        file.close(); // ファイルを閉じる
    } else {
        std::cerr << "ファイルを開けませんでした。" << std::endl; // エラーメッセージ
    }

}

template<typename T>
void LoadFileAsPair (string fileName, string type, vector<pair<T, T>>& v){
    ifstream file(fileName); // CSVファイルを開く
    string line; // 行を格納する変数

    // ファイルが開けたか確認
    if (file.is_open()) {
        while (getline(file, line)) { // 行を1行ずつ読み込む
            stringstream ss(line); // 行をストリームに変換
            string f;
            string s;

            getline(ss, f, ',');
            getline(ss, s, ',');
            try{
                if(type == "double"){
                    v.emplace_back(make_pair(stod(f), stod(s)));
                } else if(type == "int"){
                    v.emplace_back(make_pair(stoi(f), stoi(s)));
                }
            } catch (const invalid_argument& e) {
                //cerr << "変換エラー: " << f << endl;
                //cerr << "変換エラー: " << s << endl;
            } catch (const out_of_range& e) {
                cerr << "範囲外です: " << f << endl;
                cerr << "範囲外です: " << s << endl;
            }
        }
        file.close(); // ファイルを閉じる
    } else {
        std::cerr << "ファイルを開けませんでした。" << std::endl; // エラーメッセージ
    }
}

void LoadFileAsVec (string fileName, vector<int>& v){
    ifstream file(fileName); // CSVファイルを開く
    string line; // 行を格納する変数

    // ファイルが開けたか確認
    if (file.is_open()) {
        while (getline(file, line)) { // 行を1行ずつ読み込む
            stringstream ss(line); // 行をストリームに変換
            string val;

            getline(ss, val, ',');
            try{
                v.emplace_back(stoi(val));
            } catch (const invalid_argument& e) {
                //cerr << "変換エラー: " << val << endl;
            } catch (const out_of_range& e) {
                cerr << "範囲外です: " << val << endl;
            }
        }
        file.close(); // ファイルを閉じる
    } else {
        std::cerr << "ファイルを開けませんでした。" << std::endl; // エラーメッセージ
    }
}


void storeVariables(int n, string dataType, vector<map<string, double>> &feat, vector<vector<double>>& x, vector<vector<double>>& y, vector<vector<int>>& means){
    //3.1.運賃体系の入力
    map<double, double> trainFarebd;
    map<double, double> busFarebd;
    map<double, double> taxiFarebd;
    map<double, double> autobusFarebd; //SPのみ

    LoadFileAsFare("trainFarebd.csv", trainFarebd);
    LoadFileAsFare("busFarebd.csv", busFarebd);
    LoadFileAsFare("taxiFarebd.csv", taxiFarebd);
    LoadFileAsFare("autobusFarebd.csv", autobusFarebd);

    //3.2.交通手段毎の構造体配列定義, 格納
    vector<vector<Train>> trains(n, vector<Train>());
    vector<vector<Bus>> buses(n, vector<Bus>());
    vector<vector<Car>> cars(n, vector<Car>());
    vector<vector<Taxi>> taxis(n, vector<Taxi>());
    vector<vector<Motor>> motors(n, vector<Motor>());
    vector<vector<Bicycle>> bicycles(n, vector<Bicycle>());
    vector<vector<Walk>> walks(n, vector<Walk>());
    vector<vector<Autobus>> autobuses(n, vector<Autobus>()); 

    //csvデータを格納するための配列
    vector<vector<double>> trainvec;
    vector<vector<double>> busvec;
    vector<vector<double>> carvec;
    vector<vector<double>> taxivec;
    vector<vector<double>> motorvec;
    vector<vector<double>> bicyclevec;
    vector<vector<double>> walkvec;
    vector<vector<double>> autobusvec;

    LoadFile("train.csv", "double", trainvec);
    LoadFile("bus.csv", "double", busvec);
    LoadFile("car.csv", "double", carvec);
    LoadFile("taxi.csv", "double", taxivec);
    LoadFile("motor.csv", "double", motorvec);
    LoadFile("bicycle.csv", "double", bicyclevec);
    LoadFile("walk.csv", "double", walkvec);
    LoadFile("autobus.csv", "double", autobusvec);

    // 個人についてループ
    for(int i = 0; i < n; i++){
        //同じ交通手段を複数用いている場合
        //walkのみ同じ変数を使用
        int tr = 0;
        int bu = 0;
        int ca = 0;
        int ta = 0;
        int mo = 0;
        int bi = 0;
        int au = 0;

        // 使用した手段についてループ
        for(int j = 0; j < means[i].size(); j++){
            int val = means[i][j];
            switch(val){
                // 電車
                case 1:
                    trains[i].emplace_back(
                        Train{x[i][j],y[i][j], x[i][j+1],y[i][j+1],trainFarebd, trainvec[i][tr]});
                    tr ++;
                    break;
                // バス
                case 2:
                    buses[i].emplace_back(
                        Bus{x[i][j],y[i][j], x[i][j+1],y[i][j+1],busFarebd, busvec[i][bu]});
                    bu ++;
                    break;
                // 車
                case 3:
                    //csvから入力する変数は3つであることに注意
                    cars[i].emplace_back(
                        Car{x[i][j],y[i][j], x[i][j+1],y[i][j+1], carvec[i][3*ca], carvec[i][3*ca+1],carvec[i][3*ca+2]});
                    ca ++;
                    break;
                // タクシー
                case 4:
                    taxis[i].emplace_back(
                        Taxi{x[i][j],y[i][j], x[i][j+1],y[i][j+1], taxiFarebd, taxivec[i][ta]});
                    ta ++;
                    break;          
                // バイク
                case 5:
                    motors[i].emplace_back(
                        Motor{x[i][j], y[i][j], x[i][j+1],y[i][j+1], motorvec[i][3*mo], motorvec[i][3*mo+1], motorvec[i][3*mo+2]});
                    mo ++;
                    break;          
                // 自転車
                case 6:
                    bicycles[i].emplace_back(
                        Bicycle{x[i][j], y[i][j], x[i][j+1],y[i][j+1], bicyclevec[i][3*bi], bicyclevec[i][3*bi+1], bicyclevec[i][3*bi+2]});
                    bi ++;
                    break;
                // 徒歩
                case 7:
                    walks[i].emplace_back(
                        Walk{x[i][j], y[i][j], x[i][j+1],y[i][j+1], walkvec[i][0]});
                    break;
                //自動運転バス
                case 8:
                    autobuses[i].emplace_back(
                        Autobus{x[i][j], y[i][j], x[i][j+1],y[i][j+1], autobusFarebd, autobusvec[i][au]});
                        au ++;
                    break;
            }
        }
    }


    //3.3.集計
    for(int i = 0; i < n; i++){
        //予めdistanceを計算しておくことで効率化可能
        //3.3.1.費用
        double cost = 0;
        for(int j = 0; j < trains[i].size(); j++){cost += trains[i][j].getCost();}
        for(int j = 0; j < buses[i].size(); j++){cost += buses[i][j].getCost();}
        for(int j = 0; j < cars[i].size(); j++){cost += cars[i][j].getCost();}
        for(int j = 0; j < taxis[i].size(); j++){cost += taxis[i][j].getCost();}
        for(int j = 0; j < motors[i].size(); j++){cost += motors[i][j].getCost();}
        for(int j = 0; j < bicycles[i].size(); j++){cost += bicycles[i][j].getCost();}
        for(int j = 0; j < autobuses[i].size(); j++){cost += autobuses[i][j].getCost();}
        feat[i]["cost"] = cost;
        //3.3.2.総合時間
        double time = 0;
        for(int j = 0; j < trains[i].size(); j++){time += trains[i][j].getTime();}
        for(int j = 0; j < buses[i].size(); j++){time += buses[i][j].getTime();}
        for(int j = 0; j < cars[i].size(); j++){time += cars[i][j].getTime();}
        for(int j = 0; j < taxis[i].size(); j++){time += taxis[i][j].getTime();}
        for(int j = 0; j < motors[i].size(); j++){time += motors[i][j].getTime();}
        for(int j = 0; j < bicycles[i].size(); j++){time += bicycles[i][j].getTime();}
        for(int j = 0; j < walks[i].size(); j++){time += walks[i][j].getTime();}
        for(int j = 0; j < autobuses[i].size(); j++){time += autobuses[i][j].getTime();}
        feat[i]["time"] = time;

        //3.3.3.徒歩時間
        double walktime = 0;
        for(int j = 0; j < walks[i].size(); j++){walktime += walks[i][j].getTime();}
        feat[i]["walktime"] = walktime;

        //3.3.4.乗車時間
        double invehicle = 0;
        for(int j = 0; j < trains[i].size(); j++){invehicle += trains[i][j].getTime();}
        for(int j = 0; j < buses[i].size(); j++){invehicle += buses[i][j].getTime();}
        for(int j = 0; j < autobuses[i].size(); j++){invehicle += autobuses[i][j].getTime();}
        feat[i]["invehicle"] = invehicle;

        //3.3.5.アクセス距離 イグレス距離（鉄道）
        //交通手段で1または2を用いているか
        if (any_of(begin(means[i]), end(means[i]), 
                [](int w) { return w == 1 || w == 2 || w == 8; })) {
            int accessValue = means[i][0];
            switch(accessValue){
                case 3:
                    feat[i]["access"] = cars[i][0].getDistance();
                    break;
                case 4:
                    feat[i]["access"] = taxis[i][0].getDistance();
                    break;
                case 5:
                    feat[i]["access"] = motors[i][0].getDistance();
                    break;
                case 6:
                    feat[i]["access"] = bicycles[i][0].getDistance();
                    break;
                case 7:
                    feat[i]["access"] = walks[i][0].getDistance();
                    break;
                default:
                    //出発地と停留所が同じ座標の場合
                    feat[i]["access"] = 0;
                    break;
            }

            int egressValue = means[i][means[i].size() - 1];
            switch(egressValue){
                case 3:
                    feat[i]["egress"] = cars[i][cars[i].size() - 1].getDistance();
                    break;
                case 4:
                    feat[i]["egress"] = taxis[i][taxis[i].size() - 1].getDistance();
                    break;
                case 5:
                    feat[i]["egress"] = motors[i][motors[i].size() - 1].getDistance();
                    break;
                case 6:
                    feat[i]["egress"] = bicycles[i][bicycles[i].size() - 1].getDistance();
                    break;
                case 7:
                    feat[i]["egress"] = walks[i][walks[i].size() - 1].getDistance();
                    break;
                default:
                    //出発地と停留所が同じ座標の場合
                    feat[i]["egress"] = 0;
                    break;
            }
        } else{
            //D2Dの場合どちらも0
            feat[i]["access"] = 0;
            feat[i]["egress"] = 0;
        }
        //3.3.6.乗換え回数
        feat[i]["transfer"] = means[i].size() - 1;

        //3.3.7.ダミー変数 「主要交通手段」
        /*if(count(begin(means[i]), end(means[i]), 1)){
            feat[i]["train"] = 1;
        } else{
            feat[i]["train"] = 0;
        }
        if(count(begin(means[i]), end(means[i]), 8) && feat[i]["train"] == 0){
            feat[i]["autobus"] = 1;
        } else{
            feat[i]["autobus"] = 0;
        }
        if(count(begin(means[i]), end(means[i]), 2) && feat[i]["train"] == 0 && feat[i]["autobus"] == 0){
            feat[i]["bus"] = 1;
        } else{
            feat[i]["bus"] = 0;
        }
        if(any_of(begin(means[i]), end(means[i]), 
                [](int w) { return w == 3 || w == 4; }) && feat[i]["train"] == 0 && feat[i]["bus"] == 0 && feat[i]["autobus"] == 0){
            feat[i]["car"] = 1;
        } else{
            feat[i]["car"] = 0;
        }
        if(any_of(begin(means[i]), end(means[i]), 
                [](int w) { return w == 5 || w == 6; })&& feat[i]["train"] == 0 && feat[i]["bus"] == 0 && feat[i]["car"] == 0 && feat[i]["autobus"] == 0){
            feat[i]["bike"] = 1;
        } else{
            feat[i]["bike"] = 0;
        }
        if(count(begin(means[i]), end(means[i]), 7)&& feat[i]["bike"] == 0&& feat[i]["train"] == 0 && feat[i]["bus"] == 0 && feat[i]["car"] == 0 && feat[i]["autobus"] == 0){
            feat[i]["walk"] = 1;
        } else{
            feat[i]["walk"] = 0;
        }*/
        if(count(begin(means[i]), end(means[i]), 1)){
            feat[i]["train"] = 1;
        } else{
            feat[i]["train"] = 0;
        }
        if(count(begin(means[i]), end(means[i]), 2) && feat[i]["train"] == 0){
            feat[i]["bus"] = 1;
        } else{
            feat[i]["bus"] = 0;
        }
        if(any_of(begin(means[i]), end(means[i]), 
                [](int w) { return w == 3 || w == 4; }) && feat[i]["train"] == 0 && feat[i]["bus"] == 0){
            feat[i]["car"] = 1;
        } else{
            feat[i]["car"] = 0;
        }
        if(any_of(begin(means[i]), end(means[i]), 
                [](int w) { return w == 5 || w == 6; })&& feat[i]["train"] == 0 && feat[i]["bus"] == 0 && feat[i]["car"] == 0){
            feat[i]["bike"] = 1;
        } else{
            feat[i]["bike"] = 0;
        }
        if(count(begin(means[i]), end(means[i]), 7)&& feat[i]["bike"] == 0&& feat[i]["train"] == 0 && feat[i]["bus"] == 0 && feat[i]["car"] == 0){
            feat[i]["walk"] = 1;
        } else{
            feat[i]["walk"] = 0;
        }
        if(dataType == "stated"){
            feat[i]["autobus"] = 1;
        } else{
            feat[i]["autobus"] = 0;
        }

        //3.3.8.選択肢数
        int opt = 8; // 初期値は8 減少方式
        double totdis = sqrt(pow(x[i][0] - x[i][x[i].size() - 1], 2.0) +  pow(y[i][0] - y[i][y[i].size() - 1], 2.0));
        if(totdis > 5){
            if(totdis > 1){
                //直線距離1km以上で徒歩を使わない
                opt --;
            }
            //直線距離5km以上で自転車を使わない
            opt --;
        }
        if(feat[i]["licenseCar"] == 0) opt --;
        if(feat[i]["licenseBike"] == 0) opt --;
        if(feat[i]["trainAvailable"] == 0) opt --;
        if(dataType == "revealed") opt --;

        feat[i]["options"] = opt;
    }

}

int main()
{
    //1.個人の特性の入力
    //[{"placex":__, "placey":__, "gender":__, ...},{},...]
    vector<map<string, double>> rfeat; //regular
    vector<map<string, double>> afeat; //alternative
    vector<map<string, double>> sfeat; //stated(autonomous bus)
    //共通
    LoadFileAsMap("feat.csv", "double", rfeat);
    LoadFileAsMap("feat.csv", "double", afeat);
    LoadFileAsMap("feat.csv", "double", sfeat);
    //個人数
    int n = rfeat.size();

    /*確認用
    for(int i = 0; i < n; i++){
        for(const auto& key: feat[i]){
            cout << "key:" << key.first << "value:" << key.second << endl; 
        }
    }*/

    //2.RP/SPデータの入力
    DataSet data;

    /*
    以下は次のindexを使用する
    1.鉄道
    2.バス
    3.自動車
    4.タクシー
    5.バイク
    6.自転車
    7.徒歩
    (8.自動運転)
    */
    LoadFile("revealedX.csv", "double", data.revealedX);
    LoadFile("revealedY.csv", "double", data.revealedY);
    LoadFile("revealedMeans.csv", "int", data.revealedMeans);
    LoadFile("statedX.csv", "double", data.statedX);
    LoadFile("statedY.csv", "double", data.statedY);
    LoadFile("statedMeans.csv", "int", data.statedMeans);
    LoadFile("alternateX.csv", "double", data.alternateX);
    LoadFile("alternateY.csv", "double", data.alternateY);
    LoadFile("alternateMeans.csv", "int", data.alternateMeans);

    /*
    for (const auto& row : data.alternateX) {
        for (const auto& val : row) {
            std::cout << val << " ";
        }
        std::cout << std::endl;
    }*/

    //3.交通手段の入力 -> 個人属性へ追加
    //RP SP regular
    storeVariables(n,"revealed",rfeat,data.revealedX,data.revealedY,data.revealedMeans);
    //RP alternate
    storeVariables(n,"revealed",afeat,data.alternateX,data.alternateY,data.alternateMeans);
    //SP autonomous
    storeVariables(n,"stated",sfeat,data.statedX,data.statedY,data.statedMeans);
    //SP 選好結果
    vector<vector<int>> statedpreferences; //一次元配列でもよい
    LoadFile("statedIntentions.csv", "int", statedpreferences);
    for(int i = 0; i < n; i++){
        sfeat[i]["sd"] = (double)statedpreferences[i][0];    
    }
    vector<vector<int>> revealedpreferences; //一次元配列でもよい
    LoadFile("revealedIntentions.csv", "int", revealedpreferences);
    for(int i = 0; i < n; i++){
        afeat[i]["rd"] = (double)revealedpreferences[i][0];    
    }
  

    for(int i = 0; i < afeat.size(); i++){
        cout << "個人:" << i << endl;
        for(const auto& key: afeat[i]){
            cout << key.first << " : " << key.second << " "; 
        }
        cout << endl;
    }


    //4.モデルの推計

    //4.1.SPデータからμ付きパラメータ推定
    //推計に用いる変数
    //テストデータでbikeはサンプル数が少ない つまりbikeをパラメータとして導入すると収束しない
    //順番は(RPSP共通特性) -> (SP特有特性)　-> (自動運転定数項autobus)
    vector<string> sfactors = {"car","invehicle","cost","access","autobus"};
    int sclms = sfactors.size();
    //RSSP共通特性
    int rscommon  = 4;
    double teststat1 = 1.0E-4;
    double teststat2 = 1.0E-2;

    Eigen::MatrixXd scoefficients;
    scoefficients = MatrixXd::Zero(sclms, 1);//最初のパラメータ初期値は0

    Eigen::MatrixXd sgradient;
    sgradient = MatrixXd::Zero(sclms, 1);

    Eigen::MatrixXd shessian;
    shessian = MatrixXd::Zero(sclms, sclms);

    int scnt = 0;
    while(true){
        //勾配ベクトルの要素 (i,1)
        for(int i = 0; i < sclms; i++){
            double fac = 0;
            //個人 j
            for(int j = 0; j < n; j++){
                double diffs = 0; //θ'(X_sn - X_rn)の計算
                //属性 k (インデックス sfactors[k]と対応)
                for(int k = 0; k < sclms; k++){
                    diffs += scoefficients(k,0) * (sfeat[j][sfactors[k]] - rfeat[j][sfactors[k]]);
                }
                double qs = 1 / (1 + exp( - (diffs - log(sfeat[j]["options"]))));
                fac += (sfeat[j]["sd"] - qs) * (sfeat[j][sfactors[i]] - rfeat[j][sfactors[i]]);
            }
            sgradient(i,0) = fac;
        }   

        //ヘッセ行列の要素 (i,j)
        for(int i = 0; i < sclms; i++){
            for(int j = 0; j < sclms; j++){
                double fac = 0;
                for(int k = 0; k < n; k ++){
                    double diffs = 0; //θ'(X_sn - X_rn)の計算
                    for(int l = 0; l < sclms; l++){
                        diffs += scoefficients(l,0) * (sfeat[k][sfactors[l]] - rfeat[k][sfactors[l]]);
                    }
                    double qs = 1 / (1 + exp( - (diffs - log(sfeat[k]["options"]))));
                    double ql = 1 - qs;
                    fac += ql * qs * (sfeat[k][sfactors[i]] - rfeat[k][sfactors[i]]) * (sfeat[k][sfactors[j]] - rfeat[k][sfactors[j]]);
                }
                shessian(i,j) = - fac;
            }
        }
        MatrixXd renewedcos;
        renewedcos = scoefficients - shessian.ldlt().solve(sgradient);
        double cert1 = 0;
        for(int i = 0; i < sclms; i++){
            cert1 += pow(renewedcos(i,0) - scoefficients(i,0), 2.0);
        }
        cout << "scnt:" << scnt << endl;
        cout << "cert1:" << cert1 << endl;
        double cert2 = 0;
        for(int i = 0; i < sclms; i++){
            if(scoefficients(i,0) == 0) continue;//実質的に無限大と見ているのと等しい
            cert2 += (renewedcos(i,0) - scoefficients(i,0)) / scoefficients(i,0);
        }
        if(sqrt(cert1) / sclms < teststat1 && cert2 < teststat2) break;

        scoefficients = renewedcos;
        scnt ++;
        if(scnt > 100) break;// 無限ループ防止
    }

    //検定量の計算も
    cout << scnt <<  scoefficients << endl;

    Eigen::MatrixXd svarcov;
    svarcov = - shessian;
    svarcov = svarcov.inverse();
    for(int i = 0; i < sclms; i++){
        double t = 0;
        double temp = (double)svarcov(i,i);
        t = scoefficients(i,0)/sqrt(svarcov(i,i));
        cout << "t" << i << ":" << t << endl;
    }

    double slc = 0;
    for(int i = 0; i < n; i++){
        slc += sfeat[i]["sd"] * log(1/(1+exp(-scoefficients(1,0) + log(sfeat[i]["options"])))) + (1 - sfeat[i]["sd"]) *  log(1/(1+exp(scoefficients(1,0) - log(sfeat[i]["options"]))));
    }

    double smaxl = 0;
    for(int i = 0; i < n; i++){
        double diffs = 0;
        for(int j = 0; j < sclms; j ++){
            diffs += scoefficients(j,0) * (sfeat[i][sfactors[j]] - rfeat[i][sfactors[j]]);
        }
        smaxl += sfeat[i]["sd"] * log(1/(1+exp(-diffs + log(sfeat[i]["options"])))) + (1 - sfeat[i]["sd"]) * log(1/(1+exp(diffs - log(sfeat[i]["options"]))));
    }

    double schi0 = -2 * ( - n * log(2) - smaxl);
    double schic = -2 * (slc - smaxl);

    double scnt1 = 0;
    double scnt2 = 0;
    for(int i = 0; i < n; i++){
        double diffs = 0;
        for(int j = 0; j < sclms; j ++){
            diffs += scoefficients(j,0) * (sfeat[i][sfactors[j]] - rfeat[i][sfactors[j]]);
        }
        double qs = 1/(1+exp(-diffs + log(sfeat[i]["options"])));
        if(qs >= 0.5 && sfeat[i]["sd"] == 1 || qs < 0.5 && sfeat[i]["sd"] == 0) scnt1 ++;
        double qr = 1/(1+exp(diffs - log(sfeat[i]["options"])));
        if(qr >= 0.5 && sfeat[i]["sd"] == 0 || qr < 0.5 && sfeat[i]["sd"] == 1) scnt2 ++;
    }
    double srho = 1 - smaxl / (- n * log(2));

    cout << "N:" << n << endl;
    cout << "L(0):" << - n * log(2) << endl;
    cout << "L(c):" << slc << endl;
    cout << "L(θ):" << smaxl << endl;
    cout << "χ^2_0:" << schi0 << endl;
    cout << "χ^2_c:" << schic << endl;
    cout << "hitr1:" << scnt1 / n << endl;
    cout << "hitr2:" << scnt2 / n << endl;
    cout << "hitr:" << (scnt1 + scnt2) / (2 * n) << endl;
    cout << "ρ^2:" << srho << endl;
    cout << "ρ^2(bar):" << (n-sclms) * srho / n<< endl;

    //4.2.RPどうし比較
    //autobus定数項抜く
    //始めは必ずspで、2個目以降はRPデータにユニークな変数
    vector<string> rfactors = {"sp","walktime","egress"};
    int rclms = rfactors.size();

    Eigen::MatrixXd rcoefficients;
    rcoefficients = MatrixXd::Zero(rclms, 1);//最初のパラメータ初期値は0

    Eigen::MatrixXd rgradient;
    rgradient = MatrixXd::Zero(rclms, 1);

    Eigen::MatrixXd rhessian;
    rhessian = MatrixXd::Zero(rclms, rclms);
    int rcnt = 0;

    //STEP2(RPモデルどうしの推計の準備)
    for(int i = 0; i < n; i++){
        afeat[i]["sp"] = 0;
        rfeat[i]["sp"] = 0;
        for(int j = 0; j < rscommon; j++){
            afeat[i]["sp"] += scoefficients(j,0) * afeat[i][sfactors[j]];
            rfeat[i]["sp"] += scoefficients(j,0) * rfeat[i][sfactors[j]]; 
        }
    }

    while(true){
        //cout << rcoefficients << endl;
        //勾配ベクトルの要素 (i,1)
        for(int i = 0; i < rclms; i++){
            double fac = 0;
            //個人 j
            for(int j = 0; j < n; j++){
                double diffs = 0; //θ'(X_sn - X_rn)の計算
                //属性 k (インデックス rfactors[k]と対応)
                for(int k = 0; k < rclms; k++){
                    diffs += rcoefficients(k,0) * (afeat[j][rfactors[k]] - rfeat[j][rfactors[k]]);
                }
                double pa = 1 / (1 + exp( - (diffs + log(rfeat[j]["options"]))));
                fac += (afeat[i]["rd"] - pa) * (afeat[j][rfactors[i]] - rfeat[j][rfactors[i]]);
            }
            rgradient(i,0) = fac;
        }   

        //ヘッセ行列の要素 (i,j)
        for(int i = 0; i < rclms; i++){
            for(int j = 0; j < rclms; j++){
                double fac = 0;
                for(int k = 0; k < n; k++){
                    double diffs = 0; //θ'(X_sn - X_rn)の計算
                    for(int l = 0; l < rclms; l++){
                        diffs += rcoefficients(l,0) * (afeat[k][rfactors[l]] - rfeat[k][rfactors[l]]);
                    }
                    double pa = 1 / (1 + exp( - ( diffs + log(rfeat[j]["options"]))));
                    double pr = 1 - pa;
                    fac += pr * pa * (afeat[k][rfactors[i]] - rfeat[k][rfactors[i]]) * (afeat[k][rfactors[j]] - rfeat[k][rfactors[j]]);
                }
                rhessian(i,j) = - fac;
            }
        }

        MatrixXd renewedcos;
        renewedcos = rcoefficients - rhessian.ldlt().solve(rgradient);
        double cert1 = 0;
        for(int i = 0; i < rclms; i++){
            cert1 += pow(renewedcos(i,0) - rcoefficients(i,0), 2.0);
        }

        double cert2 = 0;
        for(int i = 0; i < rclms; i++){
            if(rcoefficients(i,0) == 0) continue;//実質的に無限大と見ているのと等しい
            cert2 += (renewedcos(i,0) - rcoefficients(i,0)) / rcoefficients(i,0);
        }
        if(sqrt(cert1) / rclms < teststat1 && cert2 < teststat2) break;

        rcoefficients = renewedcos;
        cout << "rco:" << rcoefficients << endl;
        rcnt ++;
        if(rcnt > 100) break;// 無限ループ防止
    }

    cout << rcnt <<  rcoefficients << endl;

    Eigen::MatrixXd rvarcov;
    rvarcov = - rhessian;
    cout << rhessian << endl;
    rvarcov = rvarcov.inverse();
    for(int i = 0; i < rclms; i++){
        double t = 0;
        double temp = (double)rvarcov(i,i);
        t = rcoefficients(i,0)/sqrt(rvarcov(i,i));
        cout << "t" << i << ":" << t << endl;
    }

    double rlc = 0;
    for(int i = 0; i < n; i++){
        rlc += afeat[i]["rd"] * log(1/(1+exp(rcoefficients(0,0) - log(rfeat[i]["options"]))));
    }

    double rmaxl = 0;
    for(int i = 0; i < n; i++){
        double diffs = 0;
        for(int j = 0; j < rclms; j ++){
            diffs += rcoefficients(j,0) * (afeat[i][rfactors[j]] - rfeat[i][rfactors[j]]);
        }
        rmaxl += log(1/(1+exp(diffs - log(rfeat[i]["options"]))));
    }

    double rchi0 = -2 * ( - n * log(2) - rmaxl);
    double rchic = -2 * (rlc - rmaxl);

    double rcnt1 = 0;
    double rcnt2 = 0;
    for(int i = 0; i < n; i++){
        double diffs = 0;
        for(int j = 0; j < rclms; j ++){
            diffs += rcoefficients(j,0) * (afeat[i][rfactors[j]] - rfeat[i][rfactors[j]]);
        }
        double pa = 1/(1+exp(-diffs - log(rfeat[i]["options"])));
        if(pa >= 0.5 && afeat[i]["rd"] == 1 || pa < 0.5 && afeat[i]["rd"] == 0) rcnt1 ++;
        double pr = 1/(1+exp(diffs + log(rfeat[i]["options"])));
        if(pr >= 0.5 && afeat[i]["rd"] == 0 || pr < 0.5 && afeat[i]["rd"] == 1) rcnt2 ++;
    }
    double rrho = 1 - rmaxl / (- n * log(2));

    cout << "N:" << n << endl;
    cout << "L(0):" << - n * log(2) << endl;
    cout << "L(c):" << rlc << endl;
    cout << "L(θ):" << rmaxl << endl;
    cout << "χ^2_0:" << rchi0 << endl;
    cout << "χ^2_c:" << rchic << endl;
    cout << "hitr1:" << rcnt1 / n << endl;
    cout << "hitr2:" << rcnt2 / n << endl;
    cout << "hitr:" << (rcnt1 + rcnt2) / (2 * n) << endl;
    cout << "ρ^2:" << rrho << endl;
    cout << "ρ^2(bar):" << (n-rclms) * rrho / n<< endl;

    //rcoefficients(0,0) = λ 逆数はμ これが0から1の間に収まらない問題
    double mu = 1 / rcoefficients(0,0);
    cout << "mu:" << mu << endl;
    //便宜的
    mu = 1;
    Eigen::MatrixXd newscoeff;
    newscoeff = MatrixXd::Zero(sclms, 1);
    //β(bar), γ(bar)
    for(int i = 0; i < sclms - 1; i++){
        newscoeff(i,0) = scoefficients(i,0) / mu;
    }

    //step3
    //ここではcfeat1をQ(s)-subwayとP(a)-alternate
    //cfeat2をQ(r)-regularとP(r)-regularとしてプールする
    vector<map<string, double>> cfeat1(2*n);
    vector<map<string, double>> cfeat2(2*n);
    for(int i = 0; i < n; i++){
        for(const auto& key: sfeat[i]){
            cfeat1[i][key.first] = mu * sfeat[i][key.first];
        }
        cfeat1[i]["d"] = sfeat[i]["sd"];
        cfeat1[i]["options"] = sfeat[i]["options"];
    }
    for(int i = 0; i < n; i++){
        for(const auto& key: afeat[i]){
            cfeat1[i + n][key.first] = afeat[i][key.first];
        }
        cfeat1[i + n]["d"] = afeat[i]["rd"];
        cfeat1[i + n]["options"] = afeat[i]["options"];
    }
    for(int i = 0; i < n; i++){
        for(const auto& key: rfeat[i]){
            cfeat2[i][key.first] = rfeat[i][key.first];
        }
        cfeat2[i]["options"] = rfeat[i]["options"];
    }
    for(int i = 0; i < n; i++){
        for(const auto& key: rfeat[i]){
            cfeat2[i + n][key.first] = rfeat[i][key.first];
        }
        cfeat2[i + n]["options"] = rfeat[i]["options"];
    }

    //4.3.再計算
    //順番は統一を保つためβ,γ,autobus,α
    vector<string> cfactors;
    for(int i = 0; i < sclms; i++){
        cfactors.emplace_back(sfactors[i]);
    }
    //最初はλ項なので抜く
    for(int i = 1; i < rclms; i++){
        cfactors.emplace_back(rfactors[i]);
    }
    int cclms = cfactors.size();
    for(int i = 0; i < cclms; i++){
        cout << cfactors[i] << endl;
    }

    Eigen::MatrixXd ccoefficients;
    ccoefficients = MatrixXd::Zero(cclms, 1);//最初のパラメータ初期値は0

    Eigen::MatrixXd cgradient;
    cgradient = MatrixXd::Zero(cclms, 1);

    Eigen::MatrixXd chessian;
    chessian = MatrixXd::Zero(cclms, cclms);
    int ccnt = 0;

    while(true){
        //勾配ベクトルの要素 (i,1)
        for(int i = 0; i < cclms; i++){
            double fac = 0;
            //個人 j
            for(int j = 0; j < 2*n; j++){
                double diffs = 0; //θ'(X_sn - X_rn)の計算
                //属性 k (インデックス cfactors[k]と対応)
                for(int k = 0; k < cclms; k++){
                    diffs += ccoefficients(k,0) * (cfeat1[j][cfactors[k]] - cfeat2[j][cfactors[k]]);
                }
                //cout << "i:" << i << "j:" << j << endl;

                double pa = 1 / (1 + exp( - (diffs + log(cfeat2[j]["options"]))));
                fac += (cfeat1[i]["d"] - pa) * (cfeat1[j][cfactors[i]] - cfeat2[j][cfactors[i]]);
            }
            cgradient(i,0) = fac;
        }   

        //ヘッセ行列の要素 (i,j)
        for(int i = 0; i < cclms; i++){
            for(int j = 0; j < cclms; j++){
                double fac = 0;
                for(int k = 0; k < 2*n; k++){
                    double diffs = 0; //θ'(X_sn - X_rn)の計算
                    for(int l = 0; l < cclms; l++){
                        diffs += ccoefficients(l,0) * (cfeat1[k][cfactors[l]] - cfeat2[k][cfactors[l]]);
                    }
                    double pa = 1 / (1 + exp( - ( diffs + log(cfeat2[j]["options"]))));
                    double pr = 1 - pa;
                    fac += pr * pa * (cfeat1[k][cfactors[i]] - cfeat2[k][cfactors[i]]) * (cfeat1[k][cfactors[j]] - cfeat2[k][cfactors[j]]);
                }
                chessian(i,j) = - fac;
            }
        }
        //cout <<"gradient:\n" << cgradient << endl;
        //cout <<"hessian\n" << chessian << endl;

        MatrixXd renewedcos;
        renewedcos = ccoefficients - chessian.ldlt().solve(cgradient);
        double cert1 = 0;
        for(int i = 0; i < cclms; i++){
            cert1 += pow(renewedcos(i,0) - ccoefficients(i,0), 2.0);
        }
        cout << "c1:" <<  cert1 << endl;

        double cert2 = 0;
        for(int i = 0; i < cclms; i++){
            if(ccoefficients(i,0) == 0) continue;//実質的に無限大と見ているのと等しい
            cert2 += (renewedcos(i,0) - ccoefficients(i,0)) / ccoefficients(i,0);
        }
        if(sqrt(cert1) / cclms < teststat1 && cert2 < teststat2) break;

        ccoefficients = renewedcos;
        ccnt ++;
        if(ccnt > 50) break;// 無限ループ防止
    }

    cout << ccnt <<  ccoefficients << endl;

    Eigen::MatrixXd cvarcov;
    cvarcov = - chessian;
    cout << chessian << endl;
    cvarcov = cvarcov.inverse();
    for(int i = 0; i < cclms; i++){
        double t = 0;
        double temp = (double)cvarcov(i,i);
        t = ccoefficients(i,0)/sqrt(cvarcov(i,i));
        cout << "t" << i << ":" << t << endl;
    }

    double clc = 0;
    for(int i = 0; i < 2 * n; i++){
        clc += cfeat1[i]["d"] * log(1/(1+exp(ccoefficients(0,0) - log(cfeat2[i]["options"]))));
    }

    double cmaxl = 0;
    for(int i = 0; i < 2 * n; i++){
        double diffs = 0;
        for(int j = 0; j < cclms; j ++){
            diffs += ccoefficients(j,0) * (cfeat1[i][cfactors[j]] - cfeat2[i][cfactors[j]]);
        }
        cmaxl += log(1/(1+exp(diffs - log(cfeat2[i]["options"]))));
    }

    double cchi0 = -2 * ( - 2 * n * log(2) - cmaxl);
    double cchic = -2 * (clc - cmaxl);

    double ccnt1 = 0;
    double ccnt2 = 0;
    for(int i = 0; i < 2 * n; i++){
        double diffs = 0;
        for(int j = 0; j < cclms; j ++){
            diffs += ccoefficients(j,0) * (cfeat1[i][cfactors[j]] - cfeat2[i][cfactors[j]]);
        }
        double pa = 1/(1+exp(-diffs - log(cfeat2[i]["options"])));
        if(pa >= 0.5 && cfeat1[i]["d"] == 1 || pa < 0.5 && cfeat1[i]["d"] == 0) ccnt1 ++;
        double pr = 1/(1+exp(diffs + log(cfeat2[i]["options"])));
        if(pr >= 0.5 && cfeat1[i]["d"] == 0 || pr < 0.5 && cfeat1[i]["d"] == 1) ccnt2 ++;
    }
    double crho = 1 - cmaxl / (- 2 * n * log(2));

    cout << "N:" << 2 * n << endl;
    cout << "L(0):" << -2 * n * log(2) << endl;
    cout << "L(c):" << clc << endl;
    cout << "L(θ):" << cmaxl << endl;
    cout << "χ^2_0:" << cchi0 << endl;
    cout << "χ^2_c:" << cchic << endl;
    cout << "hitr1:" << ccnt1 / (2 * n) << endl;
    cout << "hitr2:" << ccnt2 / (2 * n) << endl;
    cout << "hitr:" << (ccnt1 + ccnt2) / (4 * n) << endl;
    cout << "ρ^2:" << rrho << endl;
    cout << "ρ^2(bar):" << (2 * n - cclms) * rrho / 2 * n<< endl;


    //5.将来値の予測
    //x軸は縦方向とする
    double xmax = 2;
    double ymax = 3;
    //メッシュの一辺長
    double unit = 0.25;
    int zones = (xmax - unit) * (ymax - unit);

    vector<pair<double, double>> bus_route;
    vector<int> bus_hasspot;
    vector<int> bus_nearest;
    vector<int> close_trip;
    LoadFileAsPair("busroute.csv","double",bus_route);
    LoadFileAsVec("bushasspot.csv",bus_hasspot);
    LoadFileAsVec("busnearest.csv",bus_nearest);
    LoadFileAsVec("closetrip.csv",close_trip);

    //連想配列の配列closeはclose[i]["str"]でゾーンiの変数（属性）strの平均値を表す
    vector<map<string, double>> close(zones);

    map<double, double> autobusFarebd;
    LoadFileAsFare("autobusFarebd.csv", autobusFarebd);

    for(int i = 0; i < zones; i++){
        //5.1.access
        if(bus_hasspot[i]){
            //1.05はルートファクター
            close[i]["access"] = 1.05 * unit / (2 * bus_hasspot[i]);
        } else{
            //ゾーン内にspotが無い場合
            int res = i % 12;
            int quot = i / 12;
            double centerx = unit * (quot + 0.5);
            double centery = unit * (res + 0.5);
            int nearest = bus_nearest[i];
            close[i]["access"] = 1.05 * sqrt(pow(centerx - bus_route[nearest].first, 2.0) + pow(centery - bus_route[nearest].second, 2.0));
        }

        //5.2.time関連
        double a = close[i]["access"];
        double dist = 1.2 * sqrt(2) * min(xmax, ymax) + (pow(a, 2.0))/(sqrt(2) * min(xmax, ymax)) - 2 * a;
        close[i]["invehicle"] = dist / 20;
        close[i]["walktime"] = a / 4.5;
        close[i]["time"] = close[i]["invehicle"] + close[i]["walktime"];

        //5.3.その他
        for(const auto& j : autobusFarebd){
            close[i]["cost"] = j.second;
            if(dist < j.first) break;
        }
        close[i]["transfer"] = 1;
        close[i]["trainAvailable"] = 0;
        close[i]["egress"] = 0;
        close[i]["age"] = 0.5;
        //以下の数字は便宜的
        close[i]["licenseCar"] = 0.33;
        close[i]["licenseBike"] = 0.15;
        //特にautobusは徒歩との分担率の配分が難しい
        close[i]["autobus"] = 1;
        double opt = 8;
        opt -= (1 - close[i]["licenseCar"]);
        opt -= (1 - close[i]["licenseBike"]);
        opt -= ( 1- close[i]["trainAvailable"]);
        close[i]["options"] = opt;

        close[i]["trip"] = close_trip[i];
    }

    //同じことを長距離輸送についても行う 鉄道でなくても長距離輸送であれば代用可
    vector<pair<double, double>> train_route;
    vector<pair<int, int>> train_access;
    vector<int> train_hasspot;
    vector<int> train_nearest;
    vector<int> far_trip;
    LoadFileAsPair("trainroute.csv","double",train_route);
    LoadFileAsPair("trainaccess.csv","int",train_access);
    LoadFileAsVec("trainhasspot.csv",train_hasspot);
    LoadFileAsVec("trainnearest.csv",train_nearest);
    LoadFileAsVec("fartrip.csv",far_trip);
    
    //鉄道を使う場合の平均移動距離
    double train_length = 20;

    vector<map<string, double>> far(zones);
    map<double, double> trainFarebd;
    LoadFileAsFare("trainFarebd.csv", trainFarebd);

    for(int i = 0; i < zones; i++){
        if(bus_hasspot[i]){
            //「自動運転バスを（必ず）使う場合なのでtrain_hasspotではない
            far[i]["access"] = 1.05 * unit / (2 * bus_hasspot[i]);
        } else{
            //ゾーン内にspotが無い場合
            int res = i % 12;
            int quot = i / 12;
            double centerx = unit * (quot + 0.5);
            double centery = unit * (res + 0.5);
            int nearest = bus_nearest[i];
            far[i]["access"] = 1.05 * sqrt(pow(centerx - bus_route[nearest].first, 2.0) + pow(centery - bus_route[nearest].second, 2.0));
        }
        double ta = 0;
        if(train_hasspot[i]){
            ta = 1.05 * unit / (2 * train_hasspot[i]);
        } else{
            //ゾーン内にspotが無い場合
            int res = i % 12;
            int quot = i / 12;
            double centerx = unit * (quot + 0.5);
            double centery = unit * (res + 0.5);
            int nearest = train_nearest[i];
            ta = 1.05 * sqrt(pow(centerx - train_route[nearest].first, 2.0) + pow(centery - train_route[nearest].second, 2.0));
        }

        double a = far[i]["access"];
        int si = train_access[i].first;
        int gi = train_access[i].second;
        double b = sqrt(pow(bus_route[si].first - bus_route[gi].first, 2.0) + pow(bus_route[si].second - bus_route[gi].second, 2.0));
        double dist = 1.25 * sqrt(2) * train_length + (pow(ta, 2.0))/(sqrt(2) * train_length) - 2 * ta;
        far[i]["invehicle"] = (b / 20) + (dist / 50);
        far[i]["walktime"] = a / 4.5;
        far[i]["time"] = far[i]["invehicle"] + far[i]["walktime"];

        for(const auto& j : autobusFarebd){
            far[i]["cost"] = j.second;
            if(b < j.first) break;
        }
        int train_cost = 0;
        for(const auto& j : trainFarebd){
            train_cost = j.second;
            if(dist < j.first) break;
        }
        far[i]["cost"] += train_cost;

        far[i]["transfer"] = 3;
        far[i]["trainAvailable"] = 1;
        //便宜的にtrainだとegressが長くなることを反映
        far[i]["egress"] = 0.25;
        far[i]["age"] = 0.5;
        far[i]["licenseCar"] = 0.33;
        far[i]["licenseBike"] = 0.15;
        far[i]["autobus"] = 1;
        double opt = 8;
        opt -= (1 - far[i]["licenseCar"]);
        opt -= (1 - far[i]["licenseBike"]);
        opt -= ( 1- far[i]["trainAvailable"]);
        far[i]["options"] = opt;

        far[i]["trip"] = far_trip[i];
    }

    //5.4.trip数を乗じる
    //ccoefficientsのsclms - 1がパラメータβかつγ 自動運転バスの項は抜いている
    

    //変数が重複するので構造体の検討を
    return 0;
}