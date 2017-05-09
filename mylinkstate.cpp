#include <iostream>
#include <iomanip>
#include <vector>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <chrono>

using namespace std;
using namespace std::chrono;

typedef pair<int, pair<int, int>> Node;
typedef vector<Node> NodeList;
typedef vector<pair<int, int>> Path;
typedef vector<Path> PathList;
typedef high_resolution_clock::time_point Time;

char* INPUT_FILE;
int start_node;
bool display_intermediate;
int NUM_NODES;
const int INF = 1000000;
nanoseconds elapsed_time;
NodeList NETWORK;
PathList ALL_PATHS;
NodeList FORWARDING_TABLE;

// Forward declarations
bool load_network();
int getWeight(int, int);
vector<int> run_lsr();
void print_all_paths(vector<int>);
void print_forwarding_table();
bool spent_node(int, vector<int>);
int get_leaf(int);

int main(int argc, char* argv[]) {
    bool err = false;

    if (argc == 4) {
        // Get name of input file
        INPUT_FILE = argv[1];

        // Get start node
        start_node = atoi(argv[2]);

        // Display table?
        int table_val = atoi(argv[3]);
        if (table_val == 1) {
            display_intermediate = true;
        } else if (table_val == 0) {
            display_intermediate = false;
        } else {
            err = true;
        }

        if (!err) {
            Time start = high_resolution_clock::now();
            // Populate our data structure with out network info
            if (load_network()) {
                if (start_node <= NUM_NODES) {
                    // Run Link-State Routing algorithm
                    vector<int> node_list = run_lsr();

                    Time end = high_resolution_clock::now();
                    elapsed_time = duration_cast<nanoseconds>(end - start);

                    if (display_intermediate) {
                        // Print immediate
                        print_all_paths(node_list);
                    }

                    print_forwarding_table();

                    cout << "Execution time: " << (float) elapsed_time.count() / (float) 1000000 << " ms" << endl;
                } else {
                    err = true;
                }
            } else {
                err = true;
            }
        }
    } else {
        err = true;
    }

    if (err) {
        cout << "Error: Bad arguments" << endl;
    }

    return 0;
}

vector<int> run_lsr() {
    // Run Dikstra's algorithm
    int curr_node = start_node;
    int prev_weight = 0;
    vector<int> node_list;
    bool nodes_remaining = true;
    for (int i = 0; i < NUM_NODES; ++i) {
        Path step;

        for (int j = 1; j <= NUM_NODES; ++j) {
            if (j != start_node) {
                pair<int, int> this_node;
                this_node.first = INF + 1;
                this_node.second = INF + 1;

                if (nodes_remaining && !spent_node(j, node_list)) {
                    int curr_weight = getWeight(curr_node, j) + prev_weight;
                    int idx = j - 1;
                    if (j > start_node) {
                        idx = j - 2;
                    }
                    if (i > 0 && idx < ALL_PATHS[i - 1].size()) {
                        if (curr_weight < ALL_PATHS[i - 1][idx].second) {
                            this_node.first = curr_node;
                            this_node.second = curr_weight;
                        } else {
                            this_node.first = ALL_PATHS[i - 1][idx].first;
                            this_node.second = ALL_PATHS[i - 1][idx].second;
                        }
                    } else {
                        if (curr_weight < INF) {
                            this_node.first = curr_node;
                            this_node.second = curr_weight;
                        } else {
                            this_node.first = curr_node;
                            this_node.second = INF;
                        }
                    }
                }

                step.push_back(this_node);
            }
        }

        ALL_PATHS.push_back(step);

        if (nodes_remaining) {
            if (i == 0) {
                node_list.push_back(curr_node);
            }

            prev_weight = INF;
            bool all_nodes_spent = true;
            for (int k = 0; k < step.size(); k++) {
                if (step[k].second < prev_weight) {
                    if (k + 1 < start_node) {
                        if (!spent_node(k + 1, node_list)) {
                            curr_node = k + 1;
                            prev_weight = step[k].second;
                            all_nodes_spent = false;
                        }
                    } else {
                        if (!spent_node(k + 2, node_list)) {
                            curr_node = k + 2;
                            prev_weight = step[k].second;
                            all_nodes_spent = false;
                        }
                    }
                }
            }

            if (!all_nodes_spent) {
                node_list.push_back(curr_node);
            } else {
                nodes_remaining = false;
            }
        }
    }

    // Get the forwarding table for start_node
    for (int i = 1; i <= NUM_NODES; ++i) {
        if (i != start_node) {
            Node n;
            n.first = i;
            n.second.first = start_node;

            int leaf = get_leaf(i);
            int destination = leaf;
            while (destination != start_node) {
                leaf = destination;
                destination = get_leaf(leaf);
            }
            n.second.second = leaf;

            FORWARDING_TABLE.push_back(n);
        }
    }

    return node_list;
}

int get_leaf(int branch) {
    int leaf = 0;
    if (branch > start_node) {
        branch -= 2;
    } else {
        branch -= 1;
    }

    for (int i = 0; i < ALL_PATHS.size(); i++) {
        if (ALL_PATHS[i][branch].first == INF + 1) {
            leaf = ALL_PATHS[i - 1][branch].first;
            break;
        }
    }

    return leaf;
}

bool spent_node(int node, vector<int> node_list) {
    bool exclude_this = false;
    for (int k = 0; k < node_list.size(); k++) {
        if (node == node_list[k]) {
            exclude_this = true;
            break;
        }
    }
    return exclude_this;
}

int getWeight(int a, int b) {
    int _weight = 0;
    for (int i = 0; i < NETWORK.size(); i++) {
        if (NETWORK[i].first == a && NETWORK[i].second.first == b) {
            _weight = NETWORK[i].second.second;
        }
    }
    return _weight;
}

bool load_network() {
    bool good_file = true;
    string curr_line = "";
    ifstream infile(INPUT_FILE);

    if (infile.good()) {
        getline(infile, curr_line);
        NUM_NODES = atoi(curr_line.c_str());
        while (getline(infile, curr_line)) {
            pair<int, pair<int, int>> data;
            string s = "";
            int item_count = 0;
            for (int i = 0; i < curr_line.length(); i++) {
                if (curr_line[i] != ' ') {
                    s += curr_line[i];
                } else {
                    switch (item_count) {
                        case 0: data.first = atoi(s.c_str()); break;
                        case 1: data.second.first = atoi(s.c_str()); break;
                        default: break;
                    }
                    item_count++;
                    s = "";
                }
            }
            data.second.second = atoi(s.c_str());
            NETWORK.push_back(data);
        }
    } else {
        good_file = false;
    }

    infile.close();

    return good_file;
}

void print_all_paths(vector<int> node_list) {
    string list_of_nodes = "";
    cout << setfill('-') << setw(100) << "" << endl;
    cout << setfill(' ') << left << setw(8) << "Step" << setw(25) << "N'";
    for (int i = 1; i <= NUM_NODES; i++) {
        if (i != start_node) {
            cout << left << setw(8) << i;
        }
    }
    cout << endl;
    cout << setfill('-') << setw(100) << "" << setfill(' ') << endl;

    for (int i = 0; i < ALL_PATHS.size(); i++) {
        if (i > 0) {
            list_of_nodes += ",";
        }
        list_of_nodes += to_string(node_list[i]);
        cout << left << setw(8) << i << setw(25) << "{" + list_of_nodes + "}";
        for (int j = 0; j < ALL_PATHS[i].size(); j++) {
            int anode = ALL_PATHS[i][j].second;
            int bnode = ALL_PATHS[i][j].first;
            string _anode = "";
            string _bnode = "";
            if (anode <= INF && bnode <= INF) {
                _bnode = to_string(bnode);
                if (anode == INF) {
                    _anode = "INF";
                } else {
                    _anode = to_string(anode);
                }
            }

            string this_node = "";
            if (_anode != "" && _bnode != "") {
                this_node = _anode + "," + _bnode;
            }

            cout << left << setw(8) << this_node;
        }
        cout << endl;
    }

    cout << endl << endl;
}

void print_forwarding_table() {
    cout << "FORWARDING TABLE: " << endl;
    cout << setw(11) << "Destination" << setw(3) << " | " << setw(10) << "Link" << endl;
    cout << "___________________" << endl;
    for (int i = 0; i < FORWARDING_TABLE.size(); i++) {
        cout << setw(11) << "    " + to_string(FORWARDING_TABLE[i].first) << setw(3) << " | " << setw(10)
             << "(" + to_string(FORWARDING_TABLE[i].second.first) + "," + to_string(FORWARDING_TABLE[i].second.second) + ")" << endl;
    }

    cout << endl;
}