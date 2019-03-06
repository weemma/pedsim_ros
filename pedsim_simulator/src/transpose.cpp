#include <pedsim_simulator/transpose.h>
/// -----------------------------------------------------------
/// \function transpose_CSV
/// \brief Receives a csv dataset and transpose its values
/// -----------------------------------------------------------
void transpose_CSV( const std::string& filename )
{
    typedef std::vector <std::string> record;
    std::deque <record> table;
    std::size_t cols = 0;

    // read the file
    {
        std::ifstream f( filename );
        std::string s;
        while (std::getline( f, s ))
        {
            record r;
            std::istringstream ss( s );
            std::string cell;
            while (std::getline( ss, cell, ',' ))
                r.emplace_back( cell );
            table.emplace_back( r );
            cols = std::max <std::size_t> ( cols, r.size() );
        }
    }

    // write the file, transposing (col <--> row)
    {
        std::ofstream f( filename );
        for (std::size_t col = 0; col < cols; col++)
        {
            f << table[ 0 ][ col ];
            for (std::size_t row = 1; row < table.size(); row++)
            {
                f << ",";
                if (col < table[ row ].size()) f << table[ row ][ col ];
            }
            f << "\n";
        }
    }
}