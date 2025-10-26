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
        cout << "\t Delhi Metro Map" << endl;
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
            g.addVertex("Noida Sector 62~B");
			g.addVertex("Botanical Garden~B");
			g.addVertex("Yamuna Bank~B");
			g.addVertex("Rajiv Chowk~BY");
			g.addVertex("Vaishali~B");
			g.addVertex("Moti Nagar~B");
			g.addVertex("Janak Puri West~BO");
			g.addVertex("Dwarka Sector 21~B");
			g.addVertex("Huda City Center~Y");
			g.addVertex("Saket~Y");
			g.addVertex("Vishwavidyalaya~Y");
			g.addVertex("Chandni Chowk~Y");
			g.addVertex("New Delhi~YO");
			g.addVertex("AIIMS~Y");
			g.addVertex("Shivaji Stadium~O");
			g.addVertex("DDS Campus~O");
			g.addVertex("IGI Airport~O");
			g.addVertex("Rajouri Garden~BP");
			g.addVertex("Netaji Subhash Place~PR");
			g.addVertex("Punjabi Bagh West~P");
			
			g.addEdge("Noida Sector 62~B", "Botanical Garden~B", 8);
			g.addEdge("Botanical Garden~B", "Yamuna Bank~B", 10);
			g.addEdge("Yamuna Bank~B", "Vaishali~B", 8);
			g.addEdge("Yamuna Bank~B", "Rajiv Chowk~BY", 6);
			g.addEdge("Rajiv Chowk~BY", "Moti Nagar~B", 9);
			g.addEdge("Moti Nagar~B", "Janak Puri West~BO", 7);
			g.addEdge("Janak Puri West~BO", "Dwarka Sector 21~B", 6);
			g.addEdge("Huda City Center~Y", "Saket~Y", 15);
			g.addEdge("Saket~Y", "AIIMS~Y", 6);
			g.addEdge("AIIMS~Y", "Rajiv Chowk~BY", 7);
			g.addEdge("Rajiv Chowk~BY", "New Delhi~YO", 1);
			g.addEdge("New Delhi~YO", "Chandni Chowk~Y", 2);
			g.addEdge("Chandni Chowk~Y", "Vishwavidyalaya~Y", 5);
			g.addEdge("New Delhi~YO", "Shivaji Stadium~O", 2);
			g.addEdge("Shivaji Stadium~O", "DDS Campus~O", 7);
			g.addEdge("DDS Campus~O", "IGI Airport~O", 8);
			g.addEdge("Moti Nagar~B", "Rajouri Garden~BP", 2);
			g.addEdge("Punjabi Bagh West~P", "Rajouri Garden~BP", 2);
			g.addEdge("Punjabi Bagh West~P", "Netaji Subhash Place~PR", 3);
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
    system("clear");
    Graph_M g;
    Graph_M::Create_Metro_Map(g);

    cout << "\n\t\t\t****WELCOME TO THE METRO APP*****" << endl;

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
            cout << "Enter the source station: ";
            string sourceStation;
            cin.ignore();
            getline(cin, sourceStation);

            cout << "Enter the destination station: ";
            string destinationStation;
            getline(cin, destinationStation);

            // Your code for getting the shortest distance
            int distance = g.dijkstra(sourceStation, destinationStation, false);
            cout << "Shortest Distance from " << sourceStation << " to " << destinationStation << " is " << distance << " KM" << endl;
            break;
        }

        case 4: {
            cout << "Enter the source station: ";
            string sourceStation;
            cin.ignore();
            getline(cin, sourceStation);

            cout << "Enter the destination station: ";
            string destinationStation;
            getline(cin, destinationStation);

            // Your code for getting the shortest time
            int time = g.dijkstra(sourceStation, destinationStation, true);
            double minutes = ceil(static_cast<double>(time) / 60);
            cout << "Shortest Time from " << sourceStation << " to " << destinationStation << " is " << minutes << " minutes" << endl;
            break;
        }

        case 5: {
            cout << "Enter the source station: ";
            string sourceStation;
            cin.ignore();
            getline(cin, sourceStation);

            cout << "Enter the destination station: ";
            string destinationStation;
            getline(cin, destinationStation);

            // Your code for getting the shortest path (distance wise)
            string shortestPath = g.Get_Minimum_Distance(sourceStation, destinationStation);
            cout << "Shortest Path (Distance Wise) from " << sourceStation << " to " << destinationStation << " is:\n"
                 << shortestPath << endl;
            break;
        }

        case 6: {
            cout << "Enter the source station: ";
            string sourceStation;
            cin.ignore();
            getline(cin, sourceStation);

            cout << "Enter the destination station: ";
            string destinationStation;
            getline(cin, destinationStation);

            // Your code for getting the shortest path (time wise)
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
