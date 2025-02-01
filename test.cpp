#include <iostream>
#include <algorithm>
#include <vector>
#include <map>
#include <string>
#include <cmath>
#include <fstream>
#include <sstream>

using namespace std;

//2.1.0.自動車,二輪車 基底構造体
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
    virtual void calcDistance() {};

    void calcFuel(){
        fuel_ = dis_ * fuelpk_;
    }
    double getFuel() const{
        return fuel_;
    }
    double getDistance() const{
        return dis_;
    }
    double getCost() const{
        return park_ + fuel_;
    }
    double getTime() const{
        return dis_/speed_;
    }

};

//2.1.自動車(送迎・ライドシェア含む)
struct Car : Vehicle
{
public:
    Car(double sx, double sy, double gx, double gy, double p, double fp, double sp)
        : Vehicle(sx, sy, gx, gy, p, fp, sp) {}

    void calcDistance() override{
        double sl = sqrt(pow(gx_ - sx_, 2.0) +  pow(gy_ - sy_, 2.0));
        //ここでは直線距離の1.2倍を実際の距離とする
        dis_ = sl * 1.2;
    }
};


//2.2.バイク
struct Motor : Vehicle
{
public:
    Motor(double sx, double sy, double gx, double gy, double p, double fp, double sp)
        : Vehicle(sx, sy, gx, gy, p, fp, sp) {}

    void calcDistance() override{
        double sl = sqrt(pow(gx_ - sx_, 2.0) +  pow(gy_ - sy_, 2.0));
        //ここでは直線距離の1.1倍を実際の距離とする
        dis_ = sl * 1.1;
    }
};

//2.3.自転車
struct Bicycle : Vehicle
{
public:
    Bicycle(double sx, double sy, double gx, double gy, double p, double fp, double sp)
        : Vehicle(sx, sy, gx, gy, p, fp, sp) {}

    void calcDistance() override{
        double sl = sqrt(pow(gx_ - sx_, 2.0) +  pow(gy_ - sy_, 2.0));
        //ここでは直線距離の1.05倍を実際の距離とする
        dis_ = sl * 1.05;
    }
};


//2.4.0.公共交通機関 基底構造体
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

    virtual void calcDistance() {};

    double getDistance() const{
        return dis_;
    }

    double getCost(){
        double distance = getDistance();

        //mapの各farebd_の要素について
        for(const auto& i : farebd_){
            fare_ = i.second;
            if(distance < i.first){
                //「次の上がり幅」よりも小さかったら、その時の運賃を出力
                return i.second;
            }
        }
    }

    //通過待ち時間などは考えない
    double getTime(){
        double distance = getDistance();
        return (double)distance / speed_;
    }

};

//2.4.1.鉄道(LRTも含む)
struct Train : Transp
{
public:
    Train(double sx, double sy, double gx, double gy, map<double, double>& farebd, double speed)
        : Transp(sx,sy,gx,gy,farebd,speed) {}

    //電車のみ バスは異なる
    void calcDistance() override{
        double sl = sqrt(pow(gx_ - sx_, 2.0) +  pow(gy_ - sy_, 2.0));
        //ここでは直線距離の1.25倍を実際の距離とする
        dis_ = sl * 1.25;
    }
};

//2.5.バス
struct Bus : Transp
{
public:
    Bus(double sx, double sy, double gx, double gy, map<double, double>& farebd, double speed)
        : Transp(sx,sy,gx,gy,farebd,speed) {}

    void calcDistance() override{
        double sl = sqrt(pow(gx_ - sx_, 2.0) +  pow(gy_ - sy_, 2.0));
        //ここでは直線距離の1.1倍を実際の距離とする
        dis_ = sl * 1.1;
    }
};

//2.6.タクシー
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

    void calcDistance(){
        double sl = sqrt(pow(gx_ - sx_, 2.0) +  pow(gy_ - sy_, 2.0));
        //ここでは直線距離の1.2倍を実際の距離とする
        dis_ = sl * 1.2;
    }

    double getDistance() const{
        return dis_;
    }
    double getCost(){
        double distance = getDistance();

        //mapの各farebd_の要素について
        for(const auto& i : farebd_){
            fare_ = i.second;
            if(distance < i.first){
                //「次の上がり幅」よりも小さかったら、その時のタクシー金額を出力
                return i.second;
            }
        }
    }
    double getTime() const{
        return dis_/speed_;
    }
};

//2.7.徒歩
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

    void calcDistance(){
        double sl = sqrt(pow(gx_ - sx_, 2.0) +  pow(gy_ - sy_, 2.0));
        //ここでは直線距離の1.05倍を実際の距離とする
        dis_ = sl * 1.05;
    }

    double getDistance() const{
        return dis_;
    }
    double getTime() const{
        return dis_/speed_;
    }
};

struct DataSet {
    vector<vector<double>> revealedX;
    vector<vector<double>> revealedY;
    vector<vector<int>> revealedMeans;//ここはintでもよい
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
            cerr << "変換エラー: " << value << endl;
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
                    cerr << "変換エラー: " << value << endl;
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
                    cerr << "変換エラー: " << value << endl;
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

    /*確認用
    for(int i = 0; i < feat.size(); i++){
        for(const auto& key: feat[i]){
            cout << "key:" << key.first << "value:" << key.second << endl; 
        }
    }*/

    //2.交通手段の入力

    //3.RP/SPデータの入力
    DataSet data;

    LoadFile("revealedX.csv", "double", data.revealedX);
    LoadFile("revealedY.csv", "double", data.revealedY);
    LoadFile("revealedMeans.csv", "int", data.revealedMeans);

    /* 確認用
    for (const auto& row : revealedX) {
        for (const auto& val : row) {
            std::cout << val << " ";
        }
        std::cout << std::endl;
    }*/



    return 0;
}