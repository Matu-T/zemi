#include <iostream>
#include <algorithm>
#include <vector>
#include <map>
#include <string>
#include <cmath>
#include <fstream>
#include <sstream>

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
    /*vector<vector<double>> statedX;
    vector<vector<double>> statedY;
    vector<vector<int>> statedMeans;*/
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

int main()
{
    //1.個人の特性の入力
    //[{"placex":__, "placey":__, "gender":__, ...},{},...]
    vector<map<string, double>> feat;
    LoadFileAsMap("feat.csv", "double", feat);
    int n = feat.size();

    /*確認用
    for(int i = 0; i < feat.size(); i++){
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
    */
    LoadFile("revealedX.csv", "double", data.revealedX);
    LoadFile("revealedY.csv", "double", data.revealedY);
    LoadFile("revealedMeans.csv", "int", data.revealedMeans);

    /* 確認用
    for (const auto& row : data.revealedX) {
        for (const auto& val : row) {
            std::cout << val << " ";
        }
        std::cout << std::endl;
    }*/


    //3.交通手段の入力 -> 個人属性へ追加

    //3.1.運賃体系の入力
    map<double, double> trainFarebd;
    map<double, double> busFarebd;
    map<double, double> taxiFarebd;
    LoadFileAsFare("trainFarebd.csv", trainFarebd);
    LoadFileAsFare("busFarebd.csv", busFarebd);
    LoadFileAsFare("taxiFarebd.csv", taxiFarebd);

    //3.2.交通手段毎の構造体配列定義, 格納
    vector<vector<Train>> trains(n, vector<Train>());
    vector<vector<Bus>> buses(n, vector<Bus>());
    vector<vector<Car>> cars(n, vector<Car>());
    vector<vector<Taxi>> taxis(n, vector<Taxi>());
    vector<vector<Motor>> motors(n, vector<Motor>());
    vector<vector<Bicycle>> bicycles(n, vector<Bicycle>());
    vector<vector<Walk>> walks(n, vector<Walk>());

    //csvデータを格納するための配列
    vector<vector<double>> trainvec;
    vector<vector<double>> busvec;
    vector<vector<double>> carvec;
    vector<vector<double>> taxivec;
    vector<vector<double>> motorvec;
    vector<vector<double>> bicyclevec;
    vector<vector<double>> walkvec;

    LoadFile("train.csv", "double", trainvec);
    LoadFile("bus.csv", "double", busvec);
    LoadFile("car.csv", "double", carvec);
    LoadFile("taxi.csv", "double", taxivec);
    LoadFile("motor.csv", "double", motorvec);
    LoadFile("bicycle.csv", "double", bicyclevec);
    LoadFile("walk.csv", "double", walkvec);

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

        // 使用した手段についてループ
        for(int j = 0; j < data.revealedMeans[i].size(); j++){
            int val = data.revealedMeans[i][j];
            switch(val){
                // 電車
                case 1:
                    trains[i].emplace_back(
                        Train{data.revealedX[i][j],data.revealedY[i][j], data.revealedX[i][j+1],data.revealedY[i][j+1],trainFarebd, trainvec[i][tr]});
                    tr ++;
                    break;
                // バス
                case 2:
                    buses[i].emplace_back(
                        Bus{data.revealedX[i][j],data.revealedY[i][j], data.revealedX[i][j+1],data.revealedY[i][j+1],busFarebd, busvec[i][bu]});
                    bu ++;
                    break;
                // 車
                case 3:
                    //csvから入力する変数は3つであることに注意
                    cars[i].emplace_back(
                        Car{data.revealedX[i][j],data.revealedY[i][j], data.revealedX[i][j+1],data.revealedY[i][j+1], carvec[i][3*ca], carvec[i][3*ca+1],carvec[i][3*ca+2]});
                    ca ++;
                    break;
                // タクシー
                case 4:
                    taxis[i].emplace_back(
                        Taxi{data.revealedX[i][j],data.revealedY[i][j], data.revealedX[i][j+1],data.revealedY[i][j+1], taxiFarebd, taxivec[i][ta]});
                    ta ++;
                    break;          
                // バイク
                case 5:
                    motors[i].emplace_back(
                        Motor{data.revealedX[i][j], data.revealedY[i][j], data.revealedX[i][j+1],data.revealedY[i][j+1], motorvec[i][3*mo], motorvec[i][3*mo+1], motorvec[i][3*mo+2]});
                    mo ++;
                    break;          
                // 自転車
                case 6:
                    bicycles[i].emplace_back(
                        Bicycle{data.revealedX[i][j], data.revealedY[i][j], data.revealedX[i][j+1],data.revealedY[i][j+1], bicyclevec[i][3*bi], bicyclevec[i][3*bi+1], bicyclevec[i][3*bi+2]});
                    bi ++;
                    break;
                // 徒歩
                case 7:
                    walks[i].emplace_back(
                        Walk{data.revealedX[i][j], data.revealedY[i][j], data.revealedX[i][j+1],data.revealedY[i][j+1], walkvec[i][0]});
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
        feat[i]["time"] = time;
        //3.3.3.乗車時間
        double invehicle = 0;
        for(int j = 0; j < trains[i].size(); j++){invehicle += trains[i][j].getTime();}
        for(int j = 0; j < buses[i].size(); j++){invehicle += buses[i][j].getTime();}
        feat[i]["invehicle"] = invehicle;

        //3.3.4.アクセス距離 イグレス距離（鉄道）
        //交通手段で1または2を用いているか
        if (std::any_of(begin(data.revealedMeans[i]), end(data.revealedMeans[i]), 
                [](int x) { return x == 1 || x == 2; })) {
            int accessValue = data.revealedMeans[i][0];
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

            int egressValue = data.revealedMeans[i][data.revealedMeans[i].size() - 1];
            cout << data.revealedMeans[i].size() - 1 << endl;
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
        }

        //3.3.5.乗換え回数
        feat[i]["transfer"] = data.revealedMeans[i].size() - 1;
    }

    /*for(int i = 0; i < feat.size(); i++){
        cout << "個人:" << i << endl;
        for(const auto& key: feat[i]){
            cout << key.first << " : " << key.second << " "; 
        }
        cout << endl;
    }*/

    return 0;
}