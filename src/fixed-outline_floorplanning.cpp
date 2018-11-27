#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <queue>

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <ctime>

//#define DEBUG 1

using namespace std;

typedef struct hardblock {
    int id;
    int x;
    int y;
    int width;
    int height;
    int rotate;
} HardBlock;

typedef struct terminal {
    int id;
    int x;
    int y;
} Terminal;

typedef struct node {
    int parent;
    int left_child;
    int right_child;
} Node;

typedef struct cost {
    int width;
    int height;
    double area;
    double wirelength;
    double R;
    double cost;
} Cost;

int num_hardblocks, num_terminals;
int num_nets, num_pins;
vector<HardBlock> hardblocks;
vector<vector<int>> nets;
vector<Terminal> terminals;

double white_space_ratio;
double total_block_area;
// target area = total block area * (1 + white space ratio)
double area_target;
// area, wire length normalization = initial area, wirelength
double area_norm = 0, wl_norm = 0;
// fixed outline max x coordinate
int W;

// b*-tree
int root_block = -1;
vector<Node> btree;

// horizontal contour
vector<int> contour;

bool in_fixed_outline;
// floorplan with minimum cost
int min_cost_root_block;
Cost min_cost;
vector<HardBlock> min_cost_floorplan;
vector<Node> min_cost_btree;
// floorplan in fixed outline with minimum cost
int min_cost_root_block_fixed_outline;
Cost min_cost_fixed_outline;
vector<HardBlock> min_cost_floorplan_fixed_outline;
vector<Node> min_cost_btree_fixed_outline;

void ReadHardblocksFile(string hardblocks_file)
{
    ifstream file;
    file.open(hardblocks_file);

    string temp1, temp2, str;
    file >> temp1 >> temp2 >> num_hardblocks;
    file >> temp1 >> temp2 >> num_terminals;
    file >> str;

    total_block_area = 0;
    hardblocks = vector<HardBlock>(num_hardblocks);
    for (int i = 0; i < num_hardblocks; i++) {
        getline(file, str);

        size_t pos1 = str.find("(");
        pos1 = str.find("(", pos1 + 1);
        pos1 = str.find("(", pos1 + 1);
        size_t pos2 = str.find(",");
        pos2 = str.find(",", pos2 + 1);
        pos2 = str.find(",", pos2 + 1);
        size_t pos3 = str.find(")");
        pos3 = str.find(")", pos3 + 1);
        pos3 = str.find(")", pos3 + 1);

        char buffer[10];
        int width, height;
        size_t len = str.copy(buffer, pos2 - pos1 - 1, pos1 + 1);
        buffer[len] = '\0';
        width = atoi(buffer);
        len = str.copy(buffer, pos3 - pos2 - 2, pos2 + 2);
        buffer[len] = '\0';
        height = atoi(buffer);

        hardblocks[i].id = i;
        hardblocks[i].x = -1;
        hardblocks[i].y = -1;
        hardblocks[i].width = width;
        hardblocks[i].height = height;
        hardblocks[i].rotate = 0;

        total_block_area += width * height;
    }

    area_target = total_block_area * (1 + white_space_ratio);
    W = sqrt(area_target);

    cout << "Area:             " << total_block_area << '\n';
    cout << "Target Area:      " << area_target << '\n';
    cout << "W:                " << W << '\n';
    cout << '\n';

    file.close();
}

void ReadNetsFile(string nets_file)
{
    ifstream file;
    file.open(nets_file);

    string temp1, temp2, str;
    file >> temp1 >> temp2 >> num_nets;
    file >> temp1 >> temp2 >> num_pins;

    nets = vector<vector<int>>(num_nets);
    for (int i = 0; i < num_nets; i++) {
        int degree;
        file >> temp1 >> temp2 >> degree;
        for (int j = 0; j < degree; j++) {
            file >> str;
            int id;
            if (str[0] == 'p') {
                str.erase(0, 1);
                id = atoi(str.c_str()) + num_hardblocks;
            }
            else if (str[0] == 's') {
                str.erase(0, 2);
                id = atoi(str.c_str());
            }
            nets[i].emplace_back(id);
        }
    }

    file.close();
}

void ReadTerminalsFile(string terminals_file)
{
    ifstream file;
    file.open(terminals_file);

    string str;
    int x, y;

    terminals = vector<Terminal>(num_terminals + 1);
    for (int i = 1; i <= num_terminals; i++) {
        file >> str >> x >> y;
        terminals[i].id = i;
        terminals[i].x = x;
        terminals[i].y = y;
    }

    file.close();
}

void BuildInitBtree()
{
    btree = vector<Node>(num_hardblocks);

    queue<int> bfs;
    vector<int> inserted(num_hardblocks, 0);

    root_block = rand() % num_hardblocks;
    btree[root_block].parent = -1;
    bfs.push(root_block);
    inserted[root_block] = 1;

    int left = num_hardblocks - 1;
    while (!bfs.empty()) {
        int parent = bfs.front();
        bfs.pop();

        int left_child = -1, right_child = -1;
        if (left > 0) {
            do {
                left_child = rand() % num_hardblocks;
            } while (inserted[left_child]);
            btree[parent].left_child = left_child;
            bfs.push(left_child);
            inserted[left_child] = 1;
            left--;

            if (left > 0) {
                do {
                    right_child = rand() % num_hardblocks;
                } while (inserted[right_child]);
                btree[parent].right_child = right_child;
                bfs.push(right_child);
                inserted[right_child] = 1;
                left--;
            }
        }
        btree[parent].left_child = left_child;
        btree[parent].right_child = right_child;
        if (left_child != -1)
            btree[left_child].parent = parent;
        if (right_child != -1)
            btree[right_child].parent = parent;
    }
}

void InitBtree()
{
    btree = vector<Node>(num_hardblocks);
    vector<int> inserted(num_hardblocks, 0);

    root_block = rand() % num_hardblocks;
    btree[root_block].parent = -1;
    inserted[root_block] = 1;

    int row_node = root_block;
    int col_node = root_block;
    int width = hardblocks[root_block].width;
    int inserted_cnt = 1;

    while (inserted_cnt != num_hardblocks) {
        int node;
        do {
            node = rand() % num_hardblocks;
        } while (inserted[node]);

        btree[node].left_child = -1;
        btree[node].right_child = -1;
        if (width > W) {
            btree[node].parent = row_node;
            btree[row_node].right_child = node;
            row_node = node;
            col_node = node;
            width = hardblocks[node].width;
        }
        else {
            btree[node].parent = col_node;
            btree[col_node].left_child = node;
            col_node = node;
            width += hardblocks[node].width;
        }

        inserted[node] = 1;
        inserted_cnt++;
    }
}

void BtreePreorderTraverse(int cur_node, bool left)
{
    int parent = btree[cur_node].parent;
    // left or right child of parent
    if (left) 
        hardblocks[cur_node].x = hardblocks[parent].x + hardblocks[parent].width;
    else 
        hardblocks[cur_node].x = hardblocks[parent].x;

    int x_start = hardblocks[cur_node].x;
    int x_end = x_start + hardblocks[cur_node].width;
    int y_max = 0;
    for (int i = x_start; i < x_end; i++)
        if (contour[i] > y_max)
            y_max = contour[i];

    hardblocks[cur_node].y = y_max;

    y_max += hardblocks[cur_node].height;
    for (int i = x_start; i < x_end; i++)
        contour[i] = y_max;

    if (btree[cur_node].left_child != -1)
        BtreePreorderTraverse(btree[cur_node].left_child, true);
    if (btree[cur_node].right_child != -1)
        BtreePreorderTraverse(btree[cur_node].right_child, false);
}

void BtreeToFloorplan()
{
    contour = vector<int>(W * 5, 0);
    hardblocks[root_block].x = 0;
    hardblocks[root_block].y = 0;
    for (int i = 0; i < hardblocks[root_block].width; i++)
        contour[i] = hardblocks[root_block].height;

    if (btree[root_block].left_child != -1)
        BtreePreorderTraverse(btree[root_block].left_child, true);
    if (btree[root_block].right_child != -1)
        BtreePreorderTraverse(btree[root_block].right_child, false);
}

Cost CalculateCost()
{
    BtreeToFloorplan();

    int width = 0, height = 0;
    for (int i = 0; i < num_hardblocks; i++) {
        if (hardblocks[i].x + hardblocks[i].width > width)
            width = hardblocks[i].x + hardblocks[i].width;
        if (hardblocks[i].y + hardblocks[i].height > height)
            height = hardblocks[i].y + hardblocks[i].height;
    }

    // area of current floorplan
    double floorplan_area = width * height;
    // aspect ratio of current floorplan
    double R = (double)height / width;

    // half perimeter wire length
    double wirelength = 0;
    for (const vector<int> &net : nets) {
        int x_min = width + 1, x_max = 0;
        int y_min = height + 1, y_max = 0;
        for (const int pin : net) {
            if (pin < num_hardblocks) {
                int x_center = hardblocks[pin].x + hardblocks[pin].width / 2;
                int y_center = hardblocks[pin].y + hardblocks[pin].height / 2;
                if (x_center < x_min)
                    x_min = x_center;
                if (y_center < y_min)
                    y_min = y_center;
                if (x_center > x_max)
                    x_max = x_center;
                if (y_center > y_max)
                    y_max = y_center;
            }
            else {
                const Terminal &t = terminals[pin - num_hardblocks];
                if (t.x < x_min)
                    x_min = t.x;
                if (t.y < y_min)
                    y_min = t.y;
                if (t.x > x_max)
                    x_max = t.x;
                if (t.y > y_max)
                    y_max = t.y;
            }
        }

        wirelength += (x_max - x_min) + (y_max - y_min);
    }

    Cost c;
    c.width = width;
    c.height = height;
    c.area = floorplan_area;
    c.wirelength = wirelength;
    c.R = R;

    // set normalization to initial floorplan area and wirelength
    if (area_norm == 0)
        area_norm = floorplan_area;
    if (wl_norm == 0)
        wl_norm = wirelength;

    double area_cost = c.area / area_norm;
    double wl_cost = c.wirelength / wl_norm;
    double R_cost = (1 - R) * (1 - R);
    double width_penalty = 0;
    double height_penalty = 0;
    if (width > W)
        width_penalty = ((double)width / W);
    if (height > W)
        height_penalty = ((double)height / W);
    c.cost = area_cost + wl_cost + R_cost + width_penalty + height_penalty;

#ifdef DEBUG
    cout << "Width:      " << c.width << '\n';
    cout << "Height:     " << c.height << '\n';
    cout << "Area:       " << c.area << '\n';
    cout << "Wirelength: " << c.wirelength << '\n';
    cout << "R:          " << c.R << '\n';
    cout << "Cost:       " << area_cost << " + " << wl_cost << " + " << R_cost << " + "
                           << width_penalty << " + " << height_penalty  << " = " << c.cost << '\n';
    cout << '\n';
#endif

    return c;
}

void Rotate(int node)
{
    int temp = hardblocks[node].width;
    hardblocks[node].width = hardblocks[node].height;
    hardblocks[node].height = temp;
    hardblocks[node].rotate = 1 - hardblocks[node].rotate;
}

void Swap(int node1, int node2)
{
    // swap parent
    int node1_parent = btree[node1].parent;
    if (node1_parent != -1) {
        if (btree[node1_parent].left_child == node1)
            btree[node1_parent].left_child = node2;
        else if (btree[node1_parent].right_child == node1)
            btree[node1_parent].right_child = node2;
        else {
            cout << "[Error] node not parent's child\n";
            exit(1);
        }
    }

    int node2_parent = btree[node2].parent;
    if (node2_parent != -1) {
        if (btree[node2_parent].left_child == node2)
            btree[node2_parent].left_child = node1;
        else if (btree[node2_parent].right_child == node2)
            btree[node2_parent].right_child = node1;
        else {
            cout << "[Error] node not parent's child\n";
            exit(1);
        }
    }

    btree[node1].parent = node2_parent;
    btree[node2].parent = node1_parent;

    // swap children
    int node1_left_child = btree[node1].left_child;
    int node1_right_child = btree[node1].right_child;
    btree[node1].left_child = btree[node2].left_child;
    btree[node1].right_child = btree[node2].right_child;
    btree[node2].left_child = node1_left_child;
    btree[node2].right_child = node1_right_child;

    if (btree[node1].left_child != -1)
        btree[btree[node1].left_child].parent = node1;
    if (btree[node1].right_child != -1)
        btree[btree[node1].right_child].parent = node1;
    if (btree[node2].left_child != -1)
        btree[btree[node2].left_child].parent = node2;
    if (btree[node2].right_child != -1)
        btree[btree[node2].right_child].parent = node2;

    // node1, node2 are parent and child
    if (btree[node1].parent == node1)
        btree[node1].parent = node2;
    else if (btree[node1].left_child == node1)
        btree[node1].left_child = node2;
    else if (btree[node1].right_child == node1)
        btree[node1].right_child = node2;

    if (btree[node2].parent == node2)
        btree[node2].parent = node1;
    else if (btree[node2].left_child == node2)
        btree[node2].left_child = node1;
    else if (btree[node2].right_child == node2)
        btree[node2].right_child = node1;

    // root block may change
    if (node1 == root_block)
        root_block = node2;
    else if (node2 == root_block)
        root_block = node1;
}

void Move(int node, int to_node)
{
    // delete
    if (btree[node].left_child == -1 && btree[node].right_child == -1) {
        // no children
        int parent = btree[node].parent;
        if (btree[parent].left_child == node)
            btree[parent].left_child = -1;
        else if (btree[parent].right_child == node)
            btree[parent].right_child = -1;
        else {
            cout << "[Error] node not parent's child\n";
            exit(1);
        }
    }
    else if (btree[node].left_child != -1 && btree[node].right_child != -1) {
        // two children
        do {
            bool move_left;
            if (btree[node].left_child != -1 && btree[node].right_child != -1)
                move_left = rand() % 2 == 0;
            else if (btree[node].left_child != -1)
                move_left = true;
            else
                move_left = false;
            
            if (move_left)
                Swap(node, btree[node].left_child);
            else
                Swap(node, btree[node].right_child);
        } while (btree[node].left_child != -1 || btree[node].right_child != -1);

        int parent = btree[node].parent;
        if (btree[parent].left_child == node)
            btree[parent].left_child = -1;
        else if (btree[parent].right_child == node)
            btree[parent].right_child = -1;
        else {
            cout << "[Error] node not parent's child\n";
            exit(1);
        }
    }
    else {
        // one child
        int child;
        if (btree[node].left_child != -1)
            child = btree[node].left_child;
        else
            child = btree[node].right_child;

        int parent = btree[node].parent;
        if (parent != -1) {
            if (btree[parent].left_child == node)
                btree[parent].left_child = child;
            else if (btree[parent].right_child == node)
                btree[parent].right_child = child;
            else {
                cout << "[Error] [one child] node not parent's child\n";
                exit(1);
            }
        }

        btree[child].parent = parent;

        // root block may change
        if (node == root_block)
            root_block = child;
    }

    // insert
    int random_left_right = rand() % 4;
    int child;
    if (random_left_right == 0) {
        child = btree[to_node].left_child;
        btree[node].left_child = child;
        btree[node].right_child = -1;
        btree[to_node].left_child = node;
    }
    else if (random_left_right == 0) {
        child = btree[to_node].right_child;
        btree[node].left_child = child;
        btree[node].right_child = -1;
        btree[to_node].right_child = node;
    }
    else if (random_left_right == 0) {
        child = btree[to_node].left_child;
        btree[node].left_child = -1;
        btree[node].right_child = child;
        btree[to_node].left_child = node;
    }
    else {
        child = btree[to_node].right_child;
        btree[node].left_child = -1;
        btree[node].right_child = child;
        btree[to_node].right_child = node;
    }
    btree[node].parent = to_node;
    if (child != -1)
        btree[child].parent = node;
}

void Verify(vector<HardBlock> &hb)
{
    for (int i = 0; i < num_hardblocks; i++) {
        int x_bl1 = hb[i].x;
        int y_bl1 = hb[i].y;
        int x_tr1 = x_bl1 + hb[i].width;
        int y_tr1 = y_bl1 + hb[i].height;
        for (int j = 0; j < num_hardblocks; j++) {
            if (i == j)
                continue;

            int x_bl2 = hb[j].x;
            int y_bl2 = hb[j].y;
            int x_tr2 = x_bl2 + hb[j].width;
            int y_tr2 = y_bl2 + hb[j].height;

            if (!(x_tr1 <= x_bl2 || x_bl1 >= x_tr2 || y_tr1 <= y_bl2 || y_bl1 >= y_tr2)) {
                printf("[Error] hardblocks overlapped\n");
                exit(1);
            }
        }
    }
}

void SimulatedAnnealing()
{
    min_cost = CalculateCost();
    min_cost_floorplan = hardblocks;

    const double P = 0.95;
    const double r = 0.9;
    //const double epsilon = 0.001; // coolest temperature
    const int k = 20;
    const int N = k * num_hardblocks;
    const double T0 = -min_cost.cost * num_hardblocks / log(P);

    double T = T0;
    int MT = 0;
    int uphill = 0;
    int reject = 0;
    Cost prev_cost = min_cost;
    in_fixed_outline = false;

    clock_t init_time = clock();
    clock_t time = init_time;
    const int max_seconds = (num_hardblocks / 20) * (num_hardblocks / 20);
    const int TIME_LIMIT = 1200 - 5; // 20 minutes
    int seconds = 0, runtime = 0;

    do {
        MT = 0;
        uphill = 0;
        reject = 0;

        do {
            vector<HardBlock> hardblocks_temp(hardblocks);
            vector<Node> btree_temp(btree);
            int prev_root_block = root_block;

            int M = rand() % 3;
            if (M == 0) {
                // rotate
                int node = rand() % num_hardblocks;
                Rotate(node);
            }
            else if (M == 1) {
                // swap
                int node1, node2;
                node1 = rand() % num_hardblocks;
                do {
                    node2 = rand() % num_hardblocks;
                } while (node2 == node1);
                Swap(node1, node2);
            }
            else if (M == 2) {
                // move
                int node, to_node;
                node = rand() % num_hardblocks;
                do {
                    to_node = rand() % num_hardblocks;
                } while (to_node == node || btree[node].parent == to_node);
                Move(node, to_node);
            }
            else {
                cout << "[Error] Unspecified Move\n";
                exit(1);
            }

            MT++;
            Cost cur_cost = CalculateCost();
            double delta_cost = cur_cost.cost - prev_cost.cost;
            double random = ((double)rand()) / RAND_MAX;
            if (delta_cost <= 0 || random < exp(-delta_cost / T)) {
                if (delta_cost > 0)
                    uphill++;

                // feasible solution with minimum cost
                if (cur_cost.width <= W && cur_cost.height <= W) {
                    if (in_fixed_outline) {
                        if (cur_cost.cost < min_cost_fixed_outline.cost) {
                            min_cost_root_block_fixed_outline = root_block;
                            min_cost_fixed_outline = cur_cost;
                            min_cost_floorplan_fixed_outline = hardblocks;
                            min_cost_btree_fixed_outline = btree;
                        }
                    }
                    else {
                        in_fixed_outline = true;
                        min_cost_root_block_fixed_outline = root_block;
                        min_cost_fixed_outline = cur_cost;
                        min_cost_floorplan_fixed_outline = hardblocks;
                        min_cost_btree_fixed_outline = btree;
                    }
                }

                // infeasible solution with minimum cost
                if (cur_cost.cost < min_cost.cost) {
                    min_cost_root_block = root_block;
                    min_cost = cur_cost;
                    min_cost_floorplan = hardblocks;
                    min_cost_btree = btree;
                }

                prev_cost = cur_cost;
            }
            else {
                reject++;
                root_block = prev_root_block;
                if (M == 0)
                    hardblocks = hardblocks_temp;
                else
                    btree = btree_temp;
            }
        } while (uphill <= N && MT <= 2 * N);

        T *= r;

        seconds = (clock() - time) / CLOCKS_PER_SEC;
        runtime = (clock() - init_time) / CLOCKS_PER_SEC;
        if (seconds >= max_seconds && in_fixed_outline == false) {
            cout << "Overtime " << min_cost.width << " " << min_cost.height << '\n';
            seconds = 0;
            time = clock();
            T = T0;
        }
    //} while ((float)reject / MT <= 0.95 && T >= epsilon);
    } while (seconds < max_seconds && runtime < TIME_LIMIT);

    if (in_fixed_outline) {
        cout << "Found feasible solution\n";
        cout << "Width:      " << min_cost_fixed_outline.width << '\n';
        cout << "Height:     " << min_cost_fixed_outline.height << '\n';
        cout << "Area:       " << min_cost_fixed_outline.area << '\n';
        cout << "Wirelength: " << min_cost_fixed_outline.wirelength << '\n';
        cout << "R:          " << min_cost_fixed_outline.R << '\n';
        cout << "Cost:       " << min_cost_fixed_outline.cost << '\n';
        cout << '\n';

        Verify(min_cost_floorplan_fixed_outline);
    }
    else {
        cout << "Not Found feasible solution\n";
        cout << "Width:      " << min_cost.width << '\n';
        cout << "Height:     " << min_cost.height << '\n';
        cout << "Area:       " << min_cost.area << '\n';
        cout << "Wirelength: " << min_cost.wirelength << '\n';
        cout << "R:          " << min_cost.R << '\n';
        cout << "Cost:       " << min_cost.cost << '\n';
        cout << '\n';

        Verify(min_cost_floorplan);
    }
}

void OutputFloorplan(string output_file, int wirelength, vector<HardBlock> &hb)
{
    ofstream file;
    file.open(output_file);

    file << "Wirelength " << wirelength << '\n';
    file << "Blocks\n";

    for (int i = 0; i < num_hardblocks; i++) {
        if (hb[i].rotate)
            file << "sb" << i << " " << hb[i].x << " " << hb[i].y << " " << hb[i].height << " " << hb[i].width << " 1\n";
        else
            file << "sb" << i << " " << hb[i].x << " " << hb[i].y << " " << hb[i].width << " " << hb[i].height << " 0\n";
    }

    file.close();
}

unsigned int GetRandomSeed()
{
    if (num_hardblocks == 100) {
        if (white_space_ratio == 0.1)
            return 1542894266;
        else if (white_space_ratio == 0.15)
            return 1542894588;
    }
    else if (num_hardblocks == 200) {
        if (white_space_ratio == 0.1)
            return 1542892927;
        else if (white_space_ratio == 0.15)
            return 1542892927;
    }
    else if (num_hardblocks == 300) {
        if (white_space_ratio == 0.1)
            return 1542959801;
        else if (white_space_ratio == 0.15)
            return 1542955417;
    }

    return time(NULL);
}

int main(int argc, char **argv)
{
    if (argc < 6) {
        cout << "[Usage]\n";
        cout << "    ./hw3 <.hardblocks> <.nets> <.pl> <.floorplan> <white_space_ratio>\n";
        exit(1);
    }
    string hardblocks_file = argv[1];
    string nets_file = argv[2];
    string terminals_file = argv[3];
    string floorplan_file = argv[4];
    white_space_ratio = atof(argv[5]);

    ReadHardblocksFile(hardblocks_file);
    ReadNetsFile(nets_file);
    ReadTerminalsFile(terminals_file);

    unsigned int seed = GetRandomSeed();
    //unsigned int seed = time(NULL);
    srand(seed);
    cout << "Random seed: " << seed << "\n\n";

    BuildInitBtree();
    //InitBtree();

    SimulatedAnnealing();

    if (in_fixed_outline)
        OutputFloorplan(floorplan_file, min_cost_fixed_outline.wirelength, min_cost_floorplan_fixed_outline);
    else
        OutputFloorplan(floorplan_file, min_cost.wirelength, min_cost_floorplan);

    return 0;
}
