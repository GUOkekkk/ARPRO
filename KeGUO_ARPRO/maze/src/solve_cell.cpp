// With cell-based motions
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

    // constructor from base ecn::Point
    Position(ecn::Point p) : Point(p.x, p.y) {}

    int distToParent()
    {
        // in cell-based motion, the distance to the parent is always 1
        return 1;
    }

    std::vector<PositionPtr> children()
    {
        // this method should return  all positions reachable from this one
        std::vector<PositionPtr> generated;
        std::vector<std::pair<int, int>> displacements{
            //here wo put the dx and dy
            {-1, 0},//on the left
            {0, 1},//on the right
            {1, 0},//go down
            {0, -1}//go up

        };
        for(const auto& [dx, dy] : displacements){
            // From the current position
            double new_x = x + dx;
            double new_y = y + dy;
            // Check that we have a node and not a well
            if(maze.isFree(new_x, new_y)){
                generated.push_back(
                            // Cpp 17 way to create the pointer
                            // And then the constructor
                            std::make_unique<Position>(new_x, new_y));
            }

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
    Position::maze.saveSolution("cell");
     cv::waitKey(0);

}
