// With corridor-based motions
#include <a_star.h>
#include <maze.h>

using namespace std;
using namespace ecn;

// a node is a x-y position, we move from 1 each time
class Position : public Point
{
    typedef std::unique_ptr<Position> PositionPtr;

public:
    // constructor from coordinates
    Position(int _x, int _y) : Point(_x, _y) {}

    // constructor with distance
    Position(int _x, int _y, int distance) : Point(_x, _y) {
        distance_ = distance;
        }
    int distance_;
    // constructor from base ecn::Point
    Position(ecn::Point p) : Point(p.x, p.y) {}

    int distToParent()
    {
        // in cell-based motion, the distance to the parent is always 1
        return distance_;
    }

    int countwalls(){
        // count the wall number aroun this node
        int walls=0;
        walls+= Position::maze.isFree(x,y-1);
        walls+= Position::maze.isFree(x,y+1);
        walls+= Position::maze.isFree(x-1,y);
        walls+= Position::maze.isFree(x+1,y);

        return walls;
    }

    bool is_intersections(){
        // if this node is an intersection
        return countwalls()<=1;
    }

    bool is_deadend(){
        // if this node is a deadend
        return countwalls()>=3;
    }

    bool is_corner(){
        // if this node is a corner
        bool left = Position::maze.isFree(x-1,y);
        bool right = Position::maze.isFree(x+1,y);
        bool up = Position::maze.isFree(x,y+1);
        bool down = Position::maze.isFree(x,y-1);
        if(((left and up) and (!right and !down)))
            return 1;
        else if(((left and down) and (!right and !up)))
            return 1;
        else if(((right and up) and (!left and !down)))
            return 1;
        else if(((right and down) and (!left and !up)))
            return 1;
        else
            return 0;
    }

    bool is_corridor(int i, int j){
        // if this node is a line
        if(!Position::maze.isFree(i,j))
            return false;
        else{
            if(i!=x or j!=y)
                return false;
            else{
                if(i==x){
                    while(j != y and Position::maze.isFree(i,j and Position(i,j).is_corner() and !Position(i,j).is_deadend() and !Position(i,j).is_intersections())){
                        j += (y-j)/abs(y-j);
                    }
                    return j == y;
                }
                else{
                    while(i != x and Position::maze.isFree(i,j and Position(i,j).is_corner() and !Position(i,j).is_deadend() and !Position(i,j).is_intersections())){
                        i += (x-i)/abs(x-i);
                    }
                    return i == x;
                }
            }
        }
    }

    std::vector<PositionPtr> children()
    {
        cout<<distance_<<endl;
        // this method should return  all positions reachable from this one
        std::vector<PositionPtr> generated;
        // detect the right side
        int i = 0;
        while (this->is_corridor(x,y+i)) {
            i++;
            generated.push_back(PositionPtr(new Position(x, y+i, i)));
        }
        // detect the left side
        i = 0;
        while (this->is_corridor(x,y-i)) {
            i++;
            generated.push_back(PositionPtr(new Position(x, y-i, i)));
        }
        // detect the up side
        i = 0;
        while (this->is_corridor(x+i,y)) {
            i++;
            generated.push_back(PositionPtr(new Position(x+i, y, i)));
        }
        // detect the down side
        i = 0;
        while (this->is_corridor(x-i,y)) {
            i++;
            generated.push_back(PositionPtr(new Position(x-i, y, i)));
        }
        return generated;
        }
};



int main( int argc, char **argv )
{
    // load file
    std::string filename = "../mazes/maze.png";
    if(argc == 2)
        filename = std::string(argv[1]);

    // let Point know about this maze
    Position::maze.load(filename);

    // initial and goal positions as Position's
    Position start = Position::maze.start(),
             goal = Position::maze.end();

    // call A* algorithm
    ecn::Astar(start, goal);

    // save final image
    Position::maze.saveSolution("corridor");
     cv::waitKey(0);

}
