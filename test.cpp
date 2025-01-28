#include <iostream>
#include <algorithm>
#include <vector>
#include <map>
#include <string>

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
    vector<double> spx_;//各スポットのx座標
    vector<double> spy_;//各スポットのy座標
    int si_;//出発地index
    int gi_;//到着地index
    vector<double> dxy_;//距離
    map<double, double> farebd_;//std::map farebd_(fare by distance)は、料金が変わる距離とその金額を格納fare[0] = 100 fare[0.255] = 270,...
    double fare_;
    double speed_;//平均速度
public:
    Transp(vector<double> spx, vector<double> spy, int si, int gi, map<double, double>& farebd, double speed)
     : spx_(spx), spy_(spy), si_(si), gi_(gi), dxy_(vector<double>(spx.size(),0)), farebd_(farebd), fare_(0), speed_(speed) {}

    //外から入力する場合
    void setDistance(vector<double> dxy){
        dxy_ = dxy;
    }

    virtual void calcDistance() {};

    double getDistance() const{
        return abs(dxy_[gi_] - dxy_[si_]);
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
    Train(vector<double> spx, vector<double> spy, int si, int gi,map<double, double>& farebd, double speed)
        : Transp(spx,spy,si,gi,farebd,speed) {}

    //電車のみ バスは異なる
    void calcDistance() override{
        //全て0のとき 全探索する必要はない
        if(all_of(dxy_.begin(), dxy_.end(), [](double dxy) {return dxy == 0;})){
            int length = dxy_.size();
            //dxy_[0] = 0;
            for(int i = 1; i < length; ++i){
                double eachDxy = sqrt(pow(spx_[i] - spx_[i-1], 2.0) +  pow(spy_[i] - spy_[i-1], 2.0));
                //ここでは直線距離の1.2倍を実際の距離とする
                dxy_[i] = eachDxy * 1.2;
            }
        }
    }
};

//2.5.バス
struct Bus : Transp
{
public:
    Bus(vector<double> spx, vector<double> spy, int si, int gi,map<double, double>& farebd, double speed)
        : Transp(spx,spy,si,gi,farebd,speed) {}

    void calcDistance() override{
        //全て0のとき 全探索する必要はない
        if(all_of(dxy_.begin(), dxy_.end(), [](double dxy) {return dxy == 0;})){
            int length = dxy_.size();
            //dxy_[0] = 0;
            for(int i = 1; i < length; ++i){
                double eachDxy = sqrt(pow(spx_[i] - spx_[i-1], 2.0) +  pow(spy_[i] - spy_[i-1], 2.0));
                //バスと変えるとしたら距離関数
                dxy_[i] = eachDxy * 1.2;
            }
        }
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



int main()
{
    cout << "Hello" << endl;
    //1.個人の特性の入力
    int n;
    vector<map<string, double>> feat(n);
    //2.各交通手段 基礎情報
    //2.1.自動車
    //2.1.1.駐車場代 将来的には関数
    int park = 0;
    //2.1.2.燃費
    double fuel = 0;
}