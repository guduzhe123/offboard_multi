#include "log/Chlog.hpp"
//#include <unistd.h>
#include <limits.h>


namespace chlog {
    //---------------------
    // work path
    //---------------------
    static const string K_logFilePrefix = "chlog_";
    static const bool K_enTimeStamp = true;

    //--------------------------
    string getTimeStamp(bool get_time_and_date, bool get_millisecond, bool format_for_windows) {
        using std::chrono::system_clock;
        system_clock::time_point tp = system_clock::now();
        time_t raw_time = system_clock::to_time_t(tp);

        // tm*使用完后不用delete，因为tm*是由localtime创建的，并且每个线程中会有一个
        struct tm *time_info = std::localtime(&raw_time);
        char buffer[80];

        //http://www.cplusplus.com/reference/ctime/strftime/
        //https://blog.csdn.net/netyeaxi/article/details/80470349
        if (get_time_and_date) {
            if (format_for_windows) {
                strftime(buffer, sizeof(buffer), "%Y%m%d_%H-%M-%S", time_info);
            } else {
                strftime(buffer, sizeof(buffer), "%Y%m%d_%T", time_info);
            }
            std::string str(buffer);
            if (get_millisecond) {
                str += ".";
                std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        tp.time_since_epoch());
                std::string milliseconds_str = std::to_string(ms.count() % 1000);
                if (milliseconds_str.length() < 3) {
                    milliseconds_str = std::string(3 - milliseconds_str.length(), '0') + milliseconds_str;
                }
                str += milliseconds_str;
            }

            return str;
        } else {
            //strftime(buffer, sizeof(buffer), "%H:%M:%S ", time_info);
            strftime(buffer, sizeof(buffer), "%T.", time_info);
            //tm只能到秒，毫秒需要另外获取
            std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch());
            std::string milliseconds_str = std::to_string(ms.count() % 1000);
            if (milliseconds_str.length() < 3) {
                milliseconds_str = std::string(3 - milliseconds_str.length(), '0') + milliseconds_str;
            }
            return std::string(buffer) + milliseconds_str;
        }
    }

    string getLogPath() {

        return "./";
    }

    static string l_sWorkPath = getLogPath();

    //---------------------------
    //  TChannel
    //---------------------------
    class TChannel {
    public:
        string m_sChannel;
        int m_level = CHDBG;
        int m_coutLevel = CHINFO;
        bool m_enFile = true;
        bool m_enCout = true;

        explicit TChannel(const string &sCh) : m_sChannel(sCh) {
            string sDir = l_sWorkPath;
            if (sDir.empty()) {
                std::string home_path(getenv("HOME"));
                const string iniFile = home_path + "/DroneParam.ini";
                sDir = home_path + "/.ros/";
                //sDir = "./";
            }
            string sTime = "_" + getTimeStamp(true, false, true);
            string sFile = sDir + "/" + K_logFilePrefix + sCh + sTime + ".log";
            m_ofs.open(sFile);
            if (m_ofs.is_open()) {
                //chlog::info("cu", "Chlog Opened log file:'" + sFile + "'");
            } else {
                logFileErr(sFile);
            }
        };

        //------------------
        void logOut(const string &sTag, const string &s, TE_lvl lvl) {
            string s1 = lvl == CHDIS ? s : sTag + "[" + getTimeStamp() + "]: " + s;
            //必须比设定的level低 越低越严重
            if (m_enCout && m_coutLevel >= lvl) {
                string s1_chan = "[" + m_sChannel + "]-" + s1;
                if (lvl <= CHERR) {
                    cerr << s1_chan << endl;
                    //clog << s1 << endl;
                } else {
                    cout << s1_chan << endl;
                }

            }
            //-----------------

            if (m_enFile && m_ofs.is_open()) {
                unique_lock<mutex> lk(m_mutex);
                m_ofs << s1 << endl << flush;
            }


        };
    protected:
        ofstream m_ofs;
        mutex m_mutex;

    };

    //-------------------------
    //	CChLogMng
    //-------------------------
    class CChLogMng {
    public:

        Sp<TChannel> getCh(const string &sCh) {

            auto it = m_map.find(sCh);
            if (it == m_map.end()) return nullptr;
            return (*it).second;

        }

        //----------------
        void add(const Sp<TChannel> &p) {
            string sCh = p->m_sChannel;
            m_map[sCh] = p;
        }

    protected:
        map<string, Sp<TChannel>> m_map;
        string m_sPath;

    };

    static CChLogMng l_mng;

    //---------------------------------
    //  Implementation of API
    //---------------------------------

    //--------------------
    extern void log(const string &sChannel, TE_lvl lvl, const string &s) {
        auto pCh = l_mng.getCh(sChannel);
        if (pCh == nullptr) return;
        if (pCh->m_level < lvl) return;

        //--------------------
        string sTag;
        switch (lvl) {
            case CHDIS:
                sTag = "";
                break;
            case CHINFO:
                sTag = "";
                break;
            case CHWARN:
                sTag = "Warn ";
                break;
            case CHERR:
                sTag = "Error ";
                break;
            case CHDBG:
                sTag = "Dbg ";
                break;
            case CHVERBO:
                sTag = "Verb ";
                break;
        }
        //----------------------
        pCh->logOut(sTag, s, lvl);
    }

    //-------------------------------
    extern void setLevel(const string &sChannel, TE_lvl lvl) {
        auto pCh = l_mng.getCh(sChannel);
        if (pCh == nullptr) return;
        pCh->m_level = lvl;
    }

    extern void setCoutLevel(const string &sChannel, TE_lvl lvl) {
        auto pCh = l_mng.getCh(sChannel);
        if (pCh == nullptr) {
            return;
        }
        pCh->m_coutLevel = lvl;
    }

    extern void setWorkDir(const string &sPath) { l_sWorkPath = sPath; }

    extern bool initChannel(const string &sChannel) {
        l_mng.add(makeSp<TChannel>(sChannel));
        return true;
    }

    extern void setEnlogFile(const string &sChannel, bool en) {
        auto pCh = l_mng.getCh(sChannel);
        if (pCh == nullptr) return;
        pCh->m_enFile = en;
    }

    extern void setEnCout(const string &sChannel, bool en) {
        auto pCh = l_mng.getCh(sChannel);
        if (pCh == nullptr) return;
        pCh->m_enCout = en;
    }

    //--------------------------------
    extern bool runTest() {
        return true;
    }


}

