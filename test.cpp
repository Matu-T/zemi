#include <iostream>
#include <algorithm>
#include <vector>
#include <map>
#include <string>

using namespace std;

//2.0.車共通 基底構造体
struct Vehicle
{
protected:
    double sx_;//出発地x座標
    double sy_;//出発地y座標
    double gx_;//目的地x座標
    double gy_;//目的地y座標
    double dis_;//距離
public:
    Vehicle(double sx, double sy, double gx, double gy) : sx_(sx), sy_(sy), gx_(gx), gy_(gy), dis_(0) {}

    double getDistance(){
        double sl = sqrt(pow(gx_ - sx_, 2.0) +  pow(gy_ - sy_, 2.0));
        //ここでは直線距離の1.2倍を実際の距離とする
        dis_ = sl * 1.2;
        return dis_;
    }

};

//2.1.自動車(送迎・ライドシェア含む)
struct Car : Vehicle
{
private:
    double park_;//駐車代
    double fuelpk_;//キロ当たり燃費
    double fuel_;//燃費
    double cv_;//自動車速度
public:
    //駐車代は重回帰などにより直接求めるのを避けたいが…
    Car(double sx, double sy, double gx, double gy, double p, double fp, double cv)
        : Vehicle(sx, sy, gx, gy), park_(p), fuelpk_(fp), fuel_(0), cv_(cv) {}

    double getFuel(){
        fuel_ = dis_ * fuelpk_;
        return fuel_;
    }
    double getCost() const{
        return park_ + fuel_;
    }
    double getTime() const{
        return (double)dis_ / cv_;
    }
};

//2.2.タクシー
struct Taxi : Vehicle
{
private:
    //std::map farebd_(fare by distance)は、料金が変わる距離とその金額を格納fare[0] = 100 fare[0.255] = 270,...
    map<double, double> farebd_;
    double fare_;
    double tv_;//タクシー速度
public:
    Taxi(double sx, double sy, double gx, double gy, map<double, double>& farebd, double tv)
        : Vehicle(sx, sy, gx, gy), farebd_(farebd), fare_(0), tv_(tv) {}

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
        return (double)dis_ / tv_;
    }
};

//2.3.0.二輪車 基底構造体
struct Cycle
{
protected:
    double sx_;//出発地x座標
    double sy_;//出発地y座標
    double gx_;//目的地x座標
    double gy_;//目的地y座標
    double dis_;//距離
public:
    Cycle(double sx, double sy, double gx, double gy) : sx_(sx), sy_(sy), gx_(gx), gy_(gy), dis_(0) {}

    double getDistance(){
        double sl = sqrt(pow(gx_ - sx_, 2.0) +  pow(gy_ - sy_, 2.0));
        //ここでは直線距離の1.1倍を実際の距離とする
        dis_ = sl * 1.1;
        return dis_;
    }

};

//2.3.バイク
struct Motor:Cycle
{
protected:
    double park_;//駐車代
    double fuelpk_;//キロ当たり燃費
    double fuel_;//燃費
    double mcv_;//バイク速度
public:
    Motor(double sx, double sy, double gx, double gy, double p, double fp, double f, double mcv)
        : Cycle(sx, sy, gx, gy), park_(p), fuelpk_(fp), fuel_(f), mcv_(mcv) {}

    double getFuel(){
        fuel_ = dis_ * fuelpk_;
        return fuel_;
    }
    double getCost() const{
        return park_ + fuel_;
    }
    double getTime() const{
        return (double)dis_ / mcv_;
    }
};

//2.4.自転車
struct Bicycle:Cycle
{
protected:
    double park_;//駐車代
    double fuelpk_;//キロ当たり燃費
    double fuel_;//燃費
    double bcv_;//自転車速度
public:
    Bicycle(double sx, double sy, double gx, double gy, double p, double fp, double f, double bcv)
        : Cycle(sx, sy, gx, gy), park_(p), fuelpk_(fp), fuel_(f), bcv_(bcv) {}

    double getFuel(){
        fuel_ = dis_ * fuelpk_;
        return fuel_;
    }
    double getCost() const{
        return park_ + fuel_;
    }
    double getTime() const{
        return (double)dis_ / bcv_;
    }
};

//2.5.0.公共交通機関 基底構造体
struct Transp
{
protected:
    vector<double> spx_;//各スポットのx座標
    vector<double> spy_;//各スポットのy座標
    int si_;//出発地index
    int gi_;//到着地index
    vector<double> dxy_;//距離
public:
    Transp(vector<double> spx, vector<double> spy, int si, int gi) : spx_(spx), spy_(spy), si_(si), gi_(gi), dxy_(vector<double>(spx.size(),0)) {}

    void setDistance(vector<double> dxy){
        dxy_ = dxy;
    }

};

//2.5.1.鉄道(LRTも含む)
struct Train : Transp
{
private:
    map<double, double> farebd_;
    double fare_;
    double trv_;//鉄道平均速度
public:
    Train(vector<double> spx, vector<double> spy, int si, int gi, map<double, double>& farebd, double trv)
        : Transp(spx,spy,si,gi), farebd_(farebd), fare_(0), trv_(trv) {}

    //電車のみ バスは異なる
    double getDistance(){
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
        return (double)distance / trv_;
    }
};

//2.6.バス
struct Bus : Transp
{
private:
    map<double, double> farebd_;
    double fare_;
    double bv_;//バス平均速度
public:
    Bus(vector<double> spx, vector<double> spy, int si, int gi, map<double, double>& farebd, double bv)
        : Transp(spx,spy,si,gi), farebd_(farebd), fare_(0), bv_(bv) {}

    //バス
    double getDistance(){
        //全て0のとき 全探索する必要はない
        if(all_of(dxy_.begin(), dxy_.end(), [](double dxy) {return dxy == 0;})){
            int length = dxy_.size();
            //dxy_[0] = 0;
            for(int i = 1; i < length; ++i){
                double eachDxy = sqrt(pow(spx_[i] - spx_[i-1], 2.0) +  pow(spy_[i] - spy_[i-1], 2.0));
                //電車と変更するのであれば変更する点
                dxy_[i] = eachDxy * 1.2;
            }
        }
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
        return (double)distance / bv_;
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