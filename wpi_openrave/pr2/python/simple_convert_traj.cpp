#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include <libxml/parser.h>

using namespace std;

template <class T>
bool convert_text_to_num(T& t,
                         const std::string& s,
                         std::ios_base& (*f)(std::ios_base&))
{
    std::istringstream iss(s);
    return !(iss >> f >> t).fail();
}

class openraveTrajectory
{
public:
    std::vector<Eigen::VectorXd> positions;
    std::vector<Eigen::VectorXd> velocities;
    std::vector<double> deltatime;
};

struct robot_and_dof
{
    int nb_dofs;
    std::string robot_name;
    std::string type;
};

bool fct_sort( std::pair<int,robot_and_dof> a, std::pair<int,robot_and_dof> b)
{
    return a.first < b.first;
}

void loadTrajecoryFromFile( std::string filename, openraveTrajectory& traj )
{
//    std::string dir = "/home/jmainpri/workspace/drc/wpi_openrave/hubo/matlab/";
//    std::string filename = dir + "movetraj1.txt";

    cout << "-------------------------------------------" << endl;
    cout << " load file : " << filename << endl;

    xmlDocPtr doc;
    xmlNodePtr cur;
    xmlNodePtr root;
    xmlChar* tmp;

    doc = xmlParseFile(filename.c_str());
    if(doc==NULL)
    {
        cout << "Document not parsed successfully (doc==NULL)" << endl;
        return;
    }

    root = xmlDocGetRootElement(doc);
    if (root == NULL)
    {
        cout << "Document not parsed successfully" << endl;
        xmlFreeDoc(doc);
        return;
    }

    if (xmlStrcmp(root->name, xmlCharStrdup("trajectory")))
    {
        cout << "Document of the wrong type root node not trajectory" << endl;
        xmlFreeDoc(doc);
        return;
    }

    cur = root->xmlChildrenNode->next;

    //    while (cur != NULL)
    //    {
    //        cout << cur->name << endl;
    //        cur = cur->next->next;
    //    }

    if (xmlStrcmp(cur->name, xmlCharStrdup("configuration")))
    {
        cout << "Error : no node named configuration" << endl;
        xmlFreeDoc(doc);
        return;
    }

    std::vector< std::pair<int,robot_and_dof> > offsets;

    xmlNodePtr node =  cur->xmlChildrenNode->next;

    while( node != NULL )
    {
        cout << xmlGetProp( node, xmlCharStrdup("name") ) << endl;

        robot_and_dof rd;

        offsets.push_back(std::make_pair(0,rd));

        tmp = xmlGetProp( node, xmlCharStrdup("offset") );
        if (tmp == NULL)
        {
            cout << "Error: no prop named offset" << endl;
            return;
        }
        convert_text_to_num<int>( offsets.back().first, (char*)tmp, std::dec );
        cout << offsets.back().first << endl;

        tmp = xmlGetProp( node, xmlCharStrdup("dof") );
        if (tmp == NULL)
        {
            cout << "Error: no prop named offset" << endl;
            return;
        }
        convert_text_to_num<int>( offsets.back().second.nb_dofs, (char*)tmp, std::dec );
        cout << offsets.back().second.nb_dofs << endl;

        std::stringstream ss( (char *)xmlGetProp( node, xmlCharStrdup("name") ) );
        std::string line;

        std::getline( ss, line, ' ' );
        offsets.back().second.type = line;
        cout << offsets.back().second.type << endl;

        std::getline( ss, line, ' ' );
        offsets.back().second.robot_name = line;
        cout << offsets.back().second.robot_name << endl;

        node = node->next->next;
    }

    std::sort( offsets.begin(), offsets.end(), fct_sort );

    // ------------------------------------------------

    cur = cur->next->next;

    if (xmlStrcmp(cur->name, xmlCharStrdup("data")))
    {
        cout << "Error : no node named data" << endl;
        xmlFreeDoc(doc);
        return;
    }

    tmp = xmlGetProp( cur, xmlCharStrdup("count") );
    if (tmp == NULL)
    {
        cout << "Error: no prop named count" << endl;
        return;
    }
    int count = 0;
    convert_text_to_num<int>( count, (char*)tmp, std::dec );
    cout << count << endl;

    tmp = xmlNodeGetContent( cur );
    if (tmp == NULL)
    {
        cout << "Error: no prop named count" << endl;
        return;
    }

    std::string configuration( (char*)(tmp) );
    //    cout << configuration << endl;
    std::stringstream ss( configuration );
    std::vector<double> values;
    std::string line;
    while( std::getline(ss,line,' ') )
    {
        double val;
        convert_text_to_num<double>( val, line, std::dec );
        values.push_back( val );
    }

    cout << "dofs_values.size() : " << values.size() << endl;

    xmlFreeDoc(doc);

    //std::vector<Eigen::VectorXd> positions(count);
    //std::vector<Eigen::VectorXd> velocities(count);
    //std::vector<double> deltatime(count);

    traj.positions.resize(count);
    traj.velocities.resize(count);
    traj.deltatime.resize(count);

    cout << "count : " << count << endl;

    std::string robot_name = "pr2";

    int ith_value=0;

    for(int i=0;i<count;i++)
    {
        for(int k=0;k<int(offsets.size());k++)
        {
            if( offsets[k].second.type != "deltatime" &&
                offsets[k].second.robot_name != robot_name )
            {
                ith_value += offsets[k].second.nb_dofs;
                continue;
            }

            int start = ith_value + offsets[k].first;
            int end = ith_value + offsets[k].first + offsets[k].second.nb_dofs;

            ith_value += offsets[k].second.nb_dofs;

            if( offsets[k].second.type == "joint_values" )
            {
                traj.positions[i].resize( offsets[k].second.nb_dofs );

                int l=0;
                for(int j=start;j<end;j++)
                {
                    traj.positions[i][l++] = values[j];
                }
                //cout << traj.positions[i].transpose() << endl;
            }

            if( offsets[k].second.type == "joint_velocities" )
            {
                traj.velocities[i].resize( offsets[k].second.nb_dofs );

                int l=0;
                for(int j=start;j<end;j++)
                {
                    traj.velocities[i][l++] = values[j];
                }
            }

            if( offsets[k].second.type == "deltatime" )
            {
                int l=0;
                for(int j=start;j<end;j++)
                {
                    traj.deltatime[i] = values[l++];
                }
            }
        }
    }
}

void printConfigurations( const std::vector<openraveTrajectory>& trajs)
{
    for(int i=0;i<int(trajs.size());i++)
    {
        for(int j=0;j<int(trajs[i].positions.size());j++)
        {
            cout << trajs[i].positions[j].transpose() << endl;
        }
    }
}

int main( int argc, const char* argv[] )
{
    std::vector<openraveTrajectory> trajs;
    
    Eigen::VectorXd q(57);

    std::string dir = "./";

    trajs.clear();
    trajs.resize(4);
    
    loadTrajecoryFromFile( dir + "movetraj0.txt", trajs[0] );
    loadTrajecoryFromFile( dir + "movetraj1.txt", trajs[1] );
    loadTrajecoryFromFile( dir + "movetraj2.txt", trajs[2] );
    loadTrajecoryFromFile( dir + "movetraj3.txt", trajs[3] );

    printConfigurations(trajs);
    
}

