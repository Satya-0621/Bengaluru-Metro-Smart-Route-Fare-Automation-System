
#include <iostream>
#include <unordered_map>
#include <vector>
#include <queue>
#include <stack>
#include <climits>
#include <algorithm>
#include <cmath>
#include <sstream>

using namespace std;

class Graph_M {
public:
    class Vertex {
    public:
        unordered_map<string, int> nbrs;
    };

    static unordered_map<string, Vertex> vtces;

    Graph_M() {
        vtces.clear();
    }

    int numVertex() {
        return vtces.size();
    }

    bool containsVertex(string vname) {
        return vtces.count(vname) > 0;
    }

    void addVertex(string vname) {
        Vertex vtx;
        vtces[vname] = vtx;
    }

    void removeVertex(string vname) {
        Vertex vtx = vtces[vname];
        vector<string> keys;
        for (auto& entry : vtx.nbrs) {
            keys.push_back(entry.first);
        }

        for (const string& key : keys) {
            Vertex nbrVtx = vtces[key];
            nbrVtx.nbrs.erase(vname);
        }

        vtces.erase(vname);
    }

    int numEdges() {
        int count = 0;
        for (const auto& entry : vtces) {
            Vertex vtx = entry.second;
            count += vtx.nbrs.size();
        }
        return count / 2;
    }

    bool containsEdge(string vname1, string vname2) {
        if (vtces.count(vname1) == 0 || vtces.count(vname2) == 0 || vtces[vname1].nbrs.count(vname2) == 0) {
            return false;
        }
        return true;
    }

    void addEdge(string vname1, string vname2, int value) {
        if (vtces.count(vname1) == 0 || vtces.count(vname2) == 0 || vtces[vname1].nbrs.count(vname2) > 0) {
            return;
        }
        vtces[vname1].nbrs[vname2] = value;
        vtces[vname2].nbrs[vname1] = value;
    }

    void removeEdge(string vname1, string vname2) {
        if (vtces.count(vname1) == 0 || vtces.count(vname2) == 0 || vtces[vname1].nbrs.count(vname2) == 0) {
            return;
        }
        vtces[vname1].nbrs.erase(vname2);
        vtces[vname2].nbrs.erase(vname1);
    }

    void display_Map() {
        cout << "\t Bengaluru Metro Map" << endl;
        cout << "\t------------------" << endl;
        cout << "----------------------------------------------------" << endl;
        vector<string> keys;
        for (const auto& entry : vtces) {
            keys.push_back(entry.first);
        }

        for (const string& key : keys) {
            string str = key + " =>\n";
            Vertex vtx = vtces[key];
            vector<string> vtxnbrs;
            for (const auto& nbr : vtx.nbrs) {
                vtxnbrs.push_back(nbr.first);
            }

            for (const string& nbr : vtxnbrs) {
                str += "\t" + nbr + "\t";
                if (nbr.length() < 16)
                    str += "\t";
                if (nbr.length() < 8)
                    str += "\t";
                str += to_string(vtx.nbrs[nbr]) + "\n";
            }
            cout << str;
        }

        cout << "\t------------------" << endl;
        cout << "---------------------------------------------------" << endl;
    }

    void display_Stations() {
        cout << "\n***********************************************************************\n";
        vector<string> keys;
        for (const auto& entry : vtces) {
            keys.push_back(entry.first);
        }
        int i = 1;
        for (const string& key : keys) {
            cout << i << ". " << key << endl;
            i++;
        }
        cout << "\n***********************************************************************\n";
    }

    bool hasPath(string vname1, string vname2, unordered_map<string, bool>& processed) {
        if (containsEdge(vname1, vname2)) {
            return true;
        }

        processed[vname1] = true;

        Vertex vtx = vtces[vname1];
        vector<string> nbrs;
        for (const auto& entry : vtx.nbrs) {
            nbrs.push_back(entry.first);
        }

        for (const string& nbr : nbrs) {
            if (!processed.count(nbr) || !processed[nbr]) {
                if (hasPath(nbr, vname2, processed)) {
                    return true;
                }
            }
        }

        return false;
    }

    class DijkstraPair {
    public:
        string vname;
        string psf;
        int cost;

        bool operator<(const DijkstraPair& other) const {
            return other.cost < this->cost;
        }
    };

    int dijkstra(string src, string des, bool nan) {
        int val = 0;
        vector<string> ans;
        unordered_map<string, DijkstraPair> map;

        priority_queue<DijkstraPair> heap;

        for (const auto& entry : vtces) {
            DijkstraPair np;
            np.vname = entry.first;
            np.cost = INT_MAX;

            if (entry.first == src) {
                np.cost = 0;
                np.psf = entry.first;
            }

            heap.push(np);
            map[entry.first] = np;
        }

        while (!heap.empty()) {
            DijkstraPair rp = heap.top();
            heap.pop();

            if (rp.vname == des) {
                val = rp.cost;
                break;
            }

            map.erase(rp.vname);

            ans.push_back(rp.vname);

            Vertex v = vtces[rp.vname];
            for (const auto& nbr : v.nbrs) {
                if (map.count(nbr.first)) {
                    int oc = map[nbr.first].cost;
                    Vertex k = vtces[rp.vname];
                    int nc;
                    if (nan)
                        nc = rp.cost + 120 + 40 * k.nbrs[nbr.first];
                    else
                        nc = rp.cost + k.nbrs[nbr.first];

                    if (nc < oc) {
                        DijkstraPair gp = map[nbr.first];
                        gp.psf = rp.psf + nbr.first;
                        gp.cost = nc;

                        heap.push(gp);
                    }
                }
            }
        }
        return val;
    }

    class Pair {
    public:
        string vname;
        string psf;
        int min_dis;
        int min_time;
    };

    string Get_Minimum_Distance(string src, string dst) {
        int min = INT_MAX;
        string ans = "";
        unordered_map<string, bool> processed;
        stack<Pair> stk;

        Pair sp;
        sp.vname = src;
        sp.psf = src + "  ";
        sp.min_dis = 0;
        sp.min_time = 0;

        stk.push(sp);

        while (!stk.empty()) {
            Pair rp = stk.top();
            stk.pop();

            if (processed.count(rp.vname)) {
                continue;
            }

            processed[rp.vname] = true;

            if (rp.vname == dst) {
                int temp = rp.min_dis;
                if (temp < min) {
                    ans = rp.psf;
                    min = temp;
                }
                continue;
            }

            Vertex rpvtx = vtces[rp.vname];
            vector<string> nbrs;
            for (const auto& nbr : rpvtx.nbrs) {
                nbrs.push_back(nbr.first);
            }

            for (const string& nbr : nbrs) {
                if (!processed.count(nbr)) {
                    Pair np;
                    np.vname = nbr;
                    np.psf = rp.psf + nbr + "  ";
                    np.min_dis = rp.min_dis + rpvtx.nbrs[nbr];
                    stk.push(np);
                }
            }
        }
        ans += to_string(min);
        return ans;
    }

    string Get_Minimum_Time(string src, string dst) {
        int min = INT_MAX;
        string ans = "";
        unordered_map<string, bool> processed;
        stack<Pair> stk;

        Pair sp;
        sp.vname = src;
        sp.psf = src + "  ";
        sp.min_dis = 0;
        sp.min_time = 0;

        stk.push(sp);

        while (!stk.empty()) {
            Pair rp = stk.top();
            stk.pop();

            if (processed.count(rp.vname)) {
                continue;
            }

            processed[rp.vname] = true;

            if (rp.vname == dst) {
                int temp = rp.min_time;
                if (temp < min) {
                    ans = rp.psf;
                    min = temp;
                }
                continue;
            }

            Vertex rpvtx = vtces[rp.vname];
            vector<string> nbrs;
            for (const auto& nbr : rpvtx.nbrs) {
                nbrs.push_back(nbr.first);
            }

            for (const string& nbr : nbrs) {
                if (!processed.count(nbr)) {
                    Pair np;
                    np.vname = nbr;
                    np.psf = rp.psf + nbr + "  ";
                    np.min_dis = rp.min_dis + rpvtx.nbrs[nbr];
                    np.min_time = rp.min_time + 120 + 40 * rpvtx.nbrs[nbr];
                    stk.push(np);
                }
            }
        }
        double minutes = ceil(static_cast<double>(min) / 60);
        ans += to_string(minutes);
        return ans;
    }

    vector<std::string> splitString(const string& input, const string& delimiter) {
        vector<std::string> tokens;
        istringstream stream(input);
        string token;
        
        while (std::getline(stream, token, delimiter[0])) {
            tokens.push_back(token);
        }

        return tokens;
    }

    vector<string> get_Interchanges(string str) {
        vector<string> arr;
        vector<string> res = splitString(str, "  ");
        arr.push_back(res[0]);
        int count = 0;
        for (size_t i = 1; i < res.size() - 1; i++) {
            size_t index = res[i].find('~');
            string s = res[i].substr(index + 1);

            if (s.length() == 2) {
                string prev = res[i - 1].substr(res[i - 1].find('~') + 1);
                string next = res[i + 1].substr(res[i + 1].find('~') + 1);

                if (prev == next) {
                    arr.push_back(res[i]);
                } else {
                    arr.push_back(res[i] + " ==> " + res[i + 1]);
                    i++;
                    count++;
                }
            } else {
                arr.push_back(res[i]);
            }
        }
        arr.push_back(to_string(count));
        arr.push_back(res[res.size() - 1]);
        return arr;
    }

    static void Create_Metro_Map(Graph_M& g) 
    {
        // --- Purple Line (West-East) : Kengeri~P  ...  Whitefield~P ---
        g.addVertex("Kengeri~P");
        g.addVertex("RV College~P");
        g.addVertex("Nayandahalli~P");
        g.addVertex("Mysore Road~P");
        g.addVertex("Magadi Road~P");
        g.addVertex("Vijayanagar~P");
        g.addVertex("Hosahalli~P");
        g.addVertex("Majestic~PG");      // Interchange (Majestic / Kempegowda)
        g.addVertex("MG Road~P");
        g.addVertex("Trinity~P");
        g.addVertex("Indiranagar~P");
        g.addVertex("Swami Vivekananda Road~P");
        g.addVertex("Baiyappanahalli~P");
        g.addVertex("KR Puram~P");
        g.addVertex("Whitefield~P");
        g.addVertex("Banaswadi~P");

        // --- Green Line (North-South) : BIEC~G  ...  Anjanapura~G ---
        g.addVertex("BIEC~G");
        g.addVertex("Yelahanka~G");
        g.addVertex("Hebbal~G");
        g.addVertex("Jalahalli~G");
        g.addVertex("Peenya~G");
        g.addVertex("Sampangiramnagar~G");
        g.addVertex("Mahalakshmi Layout~G");
        g.addVertex("Rajajinagar~G");
        g.addVertex("SJR Layout~G");
        g.addVertex("Banashankari~G");
        g.addVertex("Jayanagar~G");
        g.addVertex("South End Circle~G");
        g.addVertex("Lalbagh~G");
        g.addVertex("Anjanapura~G");

        // --- Purple Line edges (approx. distances in km) ---
        g.addEdge("Kengeri~P", "RV College~P", 4);
        g.addEdge("RV College~P", "Nayandahalli~P", 3);
        g.addEdge("Nayandahalli~P", "Mysore Road~P", 3);
        g.addEdge("Mysore Road~P", "Magadi Road~P", 2);
        g.addEdge("Magadi Road~P", "Vijayanagar~P", 2);
        g.addEdge("Vijayanagar~P", "Hosahalli~P", 2);
        g.addEdge("Hosahalli~P", "Majestic~PG", 4);
        g.addEdge("Majestic~PG", "MG Road~P", 2);
        g.addEdge("MG Road~P", "Trinity~P", 2);
        g.addEdge("Trinity~P", "Indiranagar~P", 3);
        g.addEdge("Indiranagar~P", "Swami Vivekananda Road~P", 3);
        g.addEdge("Swami Vivekananda Road~P", "Baiyappanahalli~P", 3);
        g.addEdge("Baiyappanahalli~P", "KR Puram~P", 5);
        g.addEdge("KR Puram~P", "Banaswadi~P", 4);
        g.addEdge("Banaswadi~P", "Whitefield~P", 12); // longer stretch to Whitefield

        // --- Green Line edges (approx. distances in km) ---
        g.addEdge("BIEC~G", "Yelahanka~G", 6);
        g.addEdge("Yelahanka~G", "Hebbal~G", 8);
        g.addEdge("Hebbal~G", "Jalahalli~G", 6);
        g.addEdge("Jalahalli~G", "Peenya~G", 5);
        g.addEdge("Peenya~G", "Sampangiramnagar~G", 4);
        g.addEdge("Sampangiramnagar~G", "Mahalakshmi Layout~G", 3);
        g.addEdge("Mahalakshmi Layout~G", "Rajajinagar~G", 2);
        g.addEdge("Rajajinagar~G", "Majestic~PG", 2);    // Majestic interchange
        g.addEdge("Majestic~PG", "SJR Layout~G", 3);
        g.addEdge("SJR Layout~G", "Banashankari~G", 4);
        g.addEdge("Banashankari~G", "Jayanagar~G", 2);
        g.addEdge("Jayanagar~G", "South End Circle~G", 2);
        g.addEdge("South End Circle~G", "Lalbagh~G", 2);
        g.addEdge("Lalbagh~G", "Anjanapura~G", 6);

        // A few cross-line edges (if needed) - optional realistic connections
        // (these can be used to simulate additional interchange branches)
        g.addEdge("Baiyappanahalli~P", "Whitefield~P", 6); // alternate east stretch
    }
};
unordered_map<string, Graph_M::Vertex> Graph_M::vtces;

vector<string> splitString(const string& str, const string& delimiter) {
    vector<string> tokens;
    size_t pos = 0;
    size_t prev = 0;
    while ((pos = str.find(delimiter, prev)) != string::npos) {
        tokens.push_back(str.substr(prev, pos - prev));
        prev = pos + delimiter.length();
    }
    tokens.push_back(str.substr(prev));
    return tokens;
}

string to_string(int value) {
    stringstream ss;
    ss << value;
    return ss.str();
}

int main() {
    // clear screen if available
#if defined(_WIN32) || defined(_WIN64)
    system("cls");
#else
    system("clear");
#endif

    Graph_M g;
    Graph_M::Create_Metro_Map(g);

    cout << "\n\t\t\t****WELCOME TO THE BENGALURU METRO APP*****" << endl;

    while (true) {
        cout << "\t\t\t\t~~LIST OF ACTIONS~~\n\n";
        cout << "1. LIST ALL THE STATIONS IN THE MAP\n";
        cout << "2. SHOW THE METRO MAP\n";
        cout << "3. GET SHORTEST DISTANCE FROM A 'SOURCE' STATION TO 'DESTINATION' STATION\n";
        cout << "4. GET SHORTEST TIME TO REACH FROM A 'SOURCE' STATION TO 'DESTINATION' STATION\n";
        cout << "5. GET SHORTEST PATH (DISTANCE WISE) TO REACH FROM A 'SOURCE' STATION TO 'DESTINATION' STATION\n";
        cout << "6. GET SHORTEST PATH (TIME WISE) TO REACH FROM A 'SOURCE' STATION TO 'DESTINATION' STATION\n";
        cout << "7. EXIT THE MENU\n";
        cout << "\nENTER YOUR CHOICE FROM THE ABOVE LIST (1 to 7) : ";

        int choice = -1;
        cin >> choice;

        cout << "\n***********************************************************\n";

        if (choice == 7) {
            break;
        }

        switch (choice) {
        case 1:
            g.display_Stations();
            break;

        case 2:
            g.display_Map();
            break;

        case 3: {
            cout << "Enter the source station (use exact name & tag, e.g. Majestic~PG): ";
            string sourceStation;
            cin.ignore();
            getline(cin, sourceStation);

            cout << "Enter the destination station (use exact name & tag): ";
            string destinationStation;
            getline(cin, destinationStation);

            int distance = g.dijkstra(sourceStation, destinationStation, false);
            cout << "Shortest Distance from " << sourceStation << " to " << destinationStation << " is " << distance << " KM" << endl;
            break;
        }

        case 4: {
            cout << "Enter the source station (use exact name & tag, e.g. Majestic~PG): ";
            string sourceStation;
            cin.ignore();
            getline(cin, sourceStation);

            cout << "Enter the destination station (use exact name & tag): ";
            string destinationStation;
            getline(cin, destinationStation);

            int time = g.dijkstra(sourceStation, destinationStation, true);
            double minutes = ceil(static_cast<double>(time) / 60);
            cout << "Shortest Time from " << sourceStation << " to " << destinationStation << " is " << minutes << " minutes" << endl;
            break;
        }

        case 5: {
            cout << "Enter the source station (use exact name & tag, e.g. Majestic~PG): ";
            string sourceStation;
            cin.ignore();
            getline(cin, sourceStation);

            cout << "Enter the destination station (use exact name & tag): ";
            string destinationStation;
            getline(cin, destinationStation);

            string shortestPath = g.Get_Minimum_Distance(sourceStation, destinationStation);
            cout << "Shortest Path (Distance Wise) from " << sourceStation << " to " << destinationStation << " is:\n"
                 << shortestPath << endl;
            break;
        }

        case 6: {
            cout << "Enter the source station (use exact name & tag, e.g. Majestic~PG): ";
            string sourceStation;
            cin.ignore();
            getline(cin, sourceStation);

            cout << "Enter the destination station (use exact name & tag): ";
            string destinationStation;
            getline(cin, destinationStation);

            string shortestPath = g.Get_Minimum_Time(sourceStation, destinationStation);
            cout << "Shortest Path (Time Wise) from " << sourceStation << " to " << destinationStation << " is:\n"
                 << shortestPath << endl;
            break;
        }

        default:
            cout << "Please enter a valid option! " << endl;
            cout << "The options you can choose are from 1 to 7. " << endl;
        }
    }

    return 0;
}
