#include"uLong.h"
#include"twobuff.h"
#include<stdio.h>
#include<iostream>
#include<string>
#include<set>
#include<list>
#include<unordered_map>
#include<unordered_set>
#include<vector>
#include<queue>
#include<memory>
#include<ctime>
#include<functional>
#include<chrono>

using namespace std;
using chrono::high_resolution_clock;

struct Node;
typedef uLong State;
typedef shared_ptr<Node> NodePtr;
typedef function<int(const State& s)> Heuristic;
int inversion = 0;
static int _size = 6;       //单色将牌的个数


int Inversion(const State& s);
void animate(std::vector<uLong>& path);
vector<uLong> genTargets();
byte* getStart();
vector<string> getStateStr(vector<State> min_path);
vector<uLong> targets;
string getStringOfState(State);

struct Node {

    Node(State s, int blankPos, NodePtr p = nullptr, int h = 0)
        : s(s), blankPos(blankPos), p(p), h(h)
    {
        if (!p)
        {
            this->g = 0;
        }
        else
        {
            int span = abs(p->blankPos - this->blankPos) - 1;
            span = span > 0 ? span : 1;
            this->g = p->g + span;
        }
        f = this->h + g;
        //显示字符串版的State,便于查看
        //str_s = getStringOfState(s);
    }

    vector<NodePtr> GetNextNodes() const {
        vector<NodePtr> nodes;
        int j = 0;
        State nextstate;
        uLong newH = this->h;
        uLong li;
        uLong l1;
        uLong l2;


        for (int i = -3; i <= 3; i++) {
            if (blankPos + i < 0 || blankPos + i > 2 * _size || !i)
                continue;
            nextstate = moveTile(this->s, blankPos, blankPos + i);

            if (h) //如果h==0,那么算法不是astar 或 astar的target已找到
            {
                newH = this->h;
                switch (i)
                {
                case -3:
                    li = getValue(s, blankPos - 3);
                    l1 = getValue(s, blankPos - 2);
                    l2 = getValue(s, blankPos - 1);
                    newH -= li - l1 + li - l2;
                    break;
                case -2:
                    li = getValue(s, blankPos - 2);
                    l1 = getValue(s, blankPos - 1);
                    newH -= li - l1;
                    break;
                case 2:
                    li = getValue(s, blankPos + 2);
                    l1 = getValue(s, blankPos + 1);
                    newH += li - l1;
                    break;
                case 3:
                    li = getValue(s, blankPos + 3);
                    l1 = getValue(s, blankPos + 2);
                    l2 = getValue(s, blankPos + 1);
                    newH += li - l1 + li - l2;
                    break;
                default: break;
                }
            }

            auto n = make_shared<Node>(nextstate, this->blankPos + i, make_shared<Node>(*this), newH);
            nodes.push_back(n);
        }
        return nodes;
    }
    static uLong moveTile(uLong state, int blankPos, int index) {
        uLong tile = getValue(state, index);       //获取index处的将牌
        state = resetZero(state, index);           //将index处的将牌置为0，移走
        return setValue(state, blankPos, tile);    //将tile放至blankPos处，然后返回
    }

    //string str_s;
    State s;
    int h;
    int f;
    int g;
    int blankPos;
    NodePtr p;
};

struct NodeCompare : public binary_function<NodePtr, NodePtr, bool> {
    bool operator()(const NodePtr& x, const NodePtr& y) const {
        return x->f > y->f;
    }
};

void ConstructPath(NodePtr node, vector<State>* path) {//将路径存到path里面
    while (node) {
        path->push_back(node->s);
        //cout << "\tf:" << node->f << " g:" << node->g << endl;
        node = node->p;
    }
    reverse(path->begin(), path->end());//所有方法都统一逆序输入至此，需要反转一下。
}

class Solver {
public:
    virtual uLong Solve(State start, vector<State>* path,
        int* opened, int* closed) = 0;
};

class AStarSolver : public Solver {
public:
    explicit AStarSolver(Heuristic heuristic) : heuristic_(heuristic) {}//返回一段距离
    uLong Solve(State start, vector<State>* path, int* opened,
        int* closed) override {
        unordered_map<State, NodePtr> o;
        unordered_set<State> c;
        priority_queue<NodePtr, vector<NodePtr>, NodeCompare> q;

        int o_max = 0;
        q.emplace(new Node(start, getZero(start), nullptr, heuristic_(start)));
        int all_nodes = 1;

        while (!q.empty()) {
            auto cur = q.top();
            q.pop();
            all_nodes++;
            for (int i = 0; i < 2 * _size + 1; i++)
            {
                if (cur->s == targets[i]) {
                    all_nodes += q.size();
                    cout << "\tgn:" << cur->g << endl;
                    cout << "\topen max size:" << o_max << endl;
                    cout << "\tall nodes:" << all_nodes << endl;

                    ConstructPath(cur, path);
                    *opened = o.size();
                    *closed = c.size();
                    return cur->g;
                }
            }


            if (!c.insert(cur->s).second) continue;
            for (const auto& n : cur->GetNextNodes()) {
                auto it = o.find(n->s);
                if (c.count(n->s)) continue;
                if (it != o.end() && n->f >= it->second->f) continue;

                o[n->s] = n;
                q.push(n);
                if (o_max < o.size()) o_max = o.size();
            }
        }
        return 0;
    }

private:
    Heuristic heuristic_;
};
class FringeSolver : public Solver {
public:
    explicit FringeSolver(Heuristic heuristic) : heuristic_(heuristic) {}//返回一段距离
    uLong Solve(State start, vector<State>* path, int* opened,
        int* closed) override {
        unordered_map<State, NodePtr> c;
        list<NodePtr> F;

        int fmin, flimit = heuristic_(start);
        NodePtr startNode = make_shared<Node>(start, getZero(start), nullptr, flimit);
        F.push_back(startNode);
        c[start] = startNode;

        list<NodePtr>::iterator iter_tmp = F.begin();
        while (!F.empty()) {
            fmin = INT_MAX;

            for (list<NodePtr>::iterator iter_n = F.begin(); iter_n != F.end(); )
            {
                NodePtr n = *iter_n;
                if (n->f > flimit)
                {
                    fmin = n->f < fmin ? n->f : fmin;
                    iter_n++;
                    continue;
                }
                for (int i = 0; i < 2 * _size + 1; i++)
                    if (n->s == targets[i])
                    {
                        ConstructPath(n, path);
                        cout << "\tgn:" << n->g << endl;
                        return n->g;
                    }
                for (const auto& child : n->GetNextNodes())
                {
                    if (c.count(child->s))
                        if (child->g >= c[child->s]->g)
                            continue;

                    iter_tmp = find(F.begin(), F.end(), child);
                    if (iter_tmp != F.end())
                        F.erase(iter_tmp);
                    F.insert(++iter_n, child);
                    ----iter_n;
                    c[child->s] = child;
                }
                iter_tmp = iter_n++;
                F.erase(iter_tmp);
            }
            flimit = fmin;
        }
        return 0;
    }


private:
    Heuristic heuristic_;
};


class FringeSolver2 : public Solver {
public:
    explicit FringeSolver2(Heuristic heuristic) : heuristic_(heuristic) {}//返回一段距离
    uLong Solve(State start, vector<State>* path, int* opened,
        int* closed) override {
        vector<NodePtr> nodesV[2];
        bool isNow = 0;

        flimit = heuristic_(start);
        NodePtr startNode = make_shared<Node>(start, getZero(start), nullptr, flimit);
        nodesV[isNow].push_back(startNode);
        C[start] = startNode;

        while (!nodesV[isNow].empty()) {
            fmin = INT_MAX;
            later = &nodesV[!isNow];

            for (auto& n : nodesV[isNow])
            {
                
                if (iterDeepen(n))
                {
                    ConstructPath(getCha, path);
                    cout << "\tgn:" << getCha->g << endl;
                    return getCha->g;
                }
            }

            nodesV[isNow].clear();
            isNow = !isNow;
            flimit = fmin;
        }
        return 0;
    }


private:
    vector<NodePtr>* later; int flimit; int fmin; unordered_map<State, NodePtr> C; NodePtr getCha = nullptr;

    //fringe search的迭代加深函数
    bool iterDeepen(NodePtr n)
    {
        if (n->f > flimit)
        {
            fmin = n->f < fmin ? n->f : fmin;
            later->push_back(n);
            return false;
        }

        for (auto& t : targets)
            if (n->s == t)
            {
                this->getCha = n;
                return true;
            }
        for (auto& child : n->GetNextNodes())
        {
            if (!C.insert(pair<State, NodePtr>(child->s, child)).second && child->g >= C[child->s]->g)
                continue;

            if (iterDeepen(child)) return true;
        }
        return false;
    }
    Heuristic heuristic_;
};

class DijkstraSolver : public Solver {
public:
    uLong Solve(State start, vector<State>* path, int* opened,
        int* closed) override {
        unordered_map<State, NodePtr> o;
        unordered_set<State> c;
        priority_queue<NodePtr, vector<NodePtr>, NodeCompare> q;

        int o_max = 0;
        q.emplace(new Node(start, getZero(start), nullptr));
        int all_nodes = 1;
        while (!q.empty()) {
            auto cur = q.top();
            q.pop();
            for (int i = 0; i < 2 * _size + 1; i++)
            {
                if (cur->s == targets[i]) {
                    cout << "\tgn:" << cur->g << endl;
                    cout << "\topen max size:" << o_max << endl;
                    cout << "\tall nodes:" << all_nodes << endl;
                    ConstructPath(cur, path);
                    *opened = o.size();
                    *closed = c.size();
                    return cur->g;
                }
            }

            if (!c.insert(cur->s).second) continue;
            for (const auto& n : cur->GetNextNodes()) {
                auto it = o.find(n->s);
                if (it != o.end() && n->f >= it->second->f) continue;
                ++all_nodes;
                o[n->s] = n;
                q.push(n);
                if (o_max < o.size()) o_max = o.size();
            }
        }
        return 0;
    }
};

void main()
{
    setMask(2);
    initDoubleBuffer();

    targets = genTargets();
    AStarSolver astar(Inversion);
    FringeSolver fringe(Inversion);
    FringeSolver2 fringe2(Inversion);
    DijkstraSolver dij;
    byte* s_byte = getStart();
    //打乱初始state
    srand((unsigned)time(0));
    random_shuffle(s_byte, s_byte + 2 * _size + 1);

    vector<State> path;
    int opened;
    int closed;
    uLong s = getBitRep(s_byte, 2 * _size + 1);

    high_resolution_clock::time_point t0;
    high_resolution_clock::time_point t1;
    chrono::duration<double> time_span;

    /*cout << "fringe:" << endl;
    t0 = high_resolution_clock::now();
    fringe.Solve(s, &path, &opened, &closed);
    t1 = high_resolution_clock::now();
    time_span = chrono::duration_cast<chrono::duration<double>>(t1 - t0);
    cout << "\tpath size:" << path.size() << endl;
    cout << "\ttime span:" << time_span.count() * 1000;*/

    cout << "fringe2:" << endl;
    path.clear();
    t0 = high_resolution_clock::now();
    fringe2.Solve(s, &path, &opened, &closed);
    t1 = high_resolution_clock::now();
    time_span = chrono::duration_cast<chrono::duration<double>>(t1 - t0);
    cout << "\tpath size:" << path.size() << endl;
    cout << "\ttime span:" << time_span.count() * 1000 << endl;

    /*cout << "dijkstra:" << endl;
    path.clear();
    t0 = high_resolution_clock::now();
    dij.Solve(s, &path, &opened, &closed);
    t1 = high_resolution_clock::now();
    time_span = chrono::duration_cast<chrono::duration<double>>(t1 - t0);
    cout << "\tpath size:" << path.size() << endl;
    cout << "\ttime span:" << time_span.count() * 1000 << endl;*/

    cout << "astar:" << endl;
    path.clear();
    t0 = high_resolution_clock::now();
    astar.Solve(s, &path, &opened, &closed);
    t1 = high_resolution_clock::now();
    time_span = chrono::duration_cast<chrono::duration<double>>(t1 - t0);
    cout << "\tpath size:" << path.size() << endl;
    cout << "\ttime span:" << time_span.count() * 1000 << endl;

    free(s_byte); s_byte = NULL;
}

void animate(std::vector<uLong>& path)
{
    for (auto& s : getStateStr(path))
    {
        showBuffer(s.data(), 2 * _size + 1);
    }
}

void merge(byte* a, int left, int mid, int right)
{	//Merge将两个有序的数组排好序
    int i = left;
    int j = mid + 1;
    int k = 0;
    byte* tmp = (byte*)malloc(sizeof(byte) * (right - left + 1));
    while (i <= mid && j <= right)
    {
        if (a[i] <= a[j])
            tmp[k++] = a[i++];
        else
        {
            tmp[k++] = a[j++];
            inversion += mid - i + 1;
        }
    }
    while (i <= mid)
        tmp[k++] = a[i++];
    while (j <= right)
        tmp[k++] = a[j++];
    //tmp[]已经有序，将tmp[]中数据复制回原数组a[]
    for (int l = 0; l < k; l++)
        a[left + l] = tmp[l];
    free(tmp);
}

void mergeSort(byte* a, int left, int right)
{
    if (left < right)
    {
        int mid = (left + right) / 2;
        mergeSort(a, left, mid);
        mergeSort(a, mid + 1, right);
        merge(a, left, mid, right);
    }
}

//计算逆序对数
int Inversion(const State& s)
{
    int n = 2 * _size + 1;
    byte* b = getValues(s);

    for (int i = 0; i < n; i++)
    {
        if (!b[i])
        {
            inversion -= i;
            break;
        }
    }
    mergeSort(b, 0, n - 1);

    free(b);
    return inversion;
}



//生成所有的目标状态
vector<uLong> genTargets()
{
    int j;
    vector<uLong> vb;
    byte* b;

    for (int i = 0; i < 2 * _size + 1; i++)
    {
        b = (byte*)malloc(sizeof(byte) * 2 * _size + 1);
        j = 0;
        *(b + i) = 0;
        for (int k = 0; k < 2 * _size + 1; k++)
        {
            if (k == i)
            {
                continue;
            }
            b[k] = j++ < _size ? 1 : 2;
        }

        vb.push_back(getBitRep(b, 2 * _size + 1));
        free(b); b = NULL;
    }

    return vb;
}

byte* getStart()
{
    int i;

    byte* b = (byte*)malloc(sizeof(byte) * 2 * _size + 1);

    for (i = 0; i < _size; i++)
    {
        *(b + i) = 2;
    }
    for (i = _size; i < 2 * _size; i++)
    {
        *(b + i) = 1;
    }
    *(b + i) = 0;

    return b;
}

/// +---+---+---+
/// +   + W + B +
/// +---+---+---+
vector<string> getStateStr(vector<State> min_path)
{
    vector<string> vs;
    string s = "";

    for (auto& p : min_path)
    {
        s = "| ";
        byte* p_byte = getValues(p);
        for (int i = 0; i < 2 * _size + 1; i++)
        {
            switch ((int)p_byte[i])
            {
            case 1:
                s += "W | ";
                break;
            case 2:
                s += "B | ";
                break;
            case 0:
                s += "  | ";
                break;
            }
        }
        free(p_byte);
        vs.push_back(s);
    }
    return vs;
}


string getStringOfState(State s)
{
    byte* s_byte = getValues(s);
    string str = "";
    for (int i = 0; i < 2 * _size + 1; i++)
    {
        str += (int)s_byte[i] + 48;
    }
    free(s_byte);
    return str;
}