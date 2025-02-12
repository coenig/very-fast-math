//============================================================================================================
// C O P Y R I G H T
//------------------------------------------------------------------------------------------------------------
/// \copyright (C) 2021 Robert Bosch GmbH.
//
// The reproduction, distribution and utilization of this file as
// well as the communication of its contents to others without express
// authorization is prohibited. Offenders will be held liable for the
// payment of damages. All rights reserved in the event of the grant
// of a patent, utility model or design.
//============================================================================================================


#include "dependency_tree.hpp"

#include <cassert>

int main(int argc, char* argv[])
{
    g_verbose = true; // activate verbose output

    DependencyNode root;
    root.identifier = "root";
    root.notes = "'I AM ROOT'";
    DependencyTree tree(root);

    assert(tree.numberTotal() == 1);
    assert(tree.numberOpen() == 1);
    assert(tree.add(root) == false);

    // extract open list
    auto open_list = tree.pop();
    assert(open_list.size() == 1);

    // make sure nothing is left open
    assert(tree.numberTotal() == 1);
    assert(tree.numberOpen() == 0);

    // add one node
    DependencyNode c1;
    c1.identifier = "c1";
    assert(tree.add(c1, "root"));
    assert(tree.numberTotal() == 2);
    std::cerr << tree.numberOpen() << std::endl;
    assert(tree.numberOpen() == 1);

    // add another node
    DependencyNode c2;
    c2.identifier = "c2";
    assert(tree.add(c2, "root"));
    assert(tree.numberTotal() == 3);
    assert(tree.numberOpen() == 2);

    // extract open list
    open_list = tree.pop();
    assert(open_list.size() == 2);

    // make sure nothing is left open
    assert(tree.numberTotal() == 3);
    assert(tree.numberOpen() == 0);

    std::cerr << "fin" << std::endl;
    return 0;
}
