#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/point_generators_3.h>
#include <CGAL/Convex_hull_d.h>
#include <CGAL/algorithm.h>
#include <CGAL/Convex_hull_d_traits_3.h>
#include <CGAL/Convex_hull_d_to_polyhedron_3.h>
#include <CGAL/Polyhedron_3.h>

#include <iterator>
#include <iostream>
#include <fstream>
#include <cassert>
#include <string>
#include <cstdio>
#include <ctime>
#include <list>

typedef CGAL::Exact_predicates_exact_constructions_kernel K;
typedef CGAL::Polyhedron_3< K> 								Polyhedron_3;
typedef K::Point_3 											Point_3;
typedef CGAL::Creator_uniform_3<double,Point_3>  			Creator;
typedef std::istream_iterator<Point_3>  					Iiter_p3;
typedef std::vector<Point_3> 								Points_3;

typedef CGAL::Convex_hull_d_traits_3<K>           			Hull_traits_3;
typedef CGAL::Convex_hull_d<Hull_traits_3>    				Convex_hull_3;



int main(int argc, char* argv[])
{	
	unsigned seed;
	Points_3 points;
	int no_of_pnts = 0;

	srand(seed);
	if( argc==3 ){
		if( strcmp( argv[1],"-generate" )==0 ){
			no_of_pnts = atoi( argv[2] );
		}
		if( no_of_pnts>0 ){
			std::cout << "Will be generating random points..." << std::endl;
			int a = rand()%1000;
			CGAL::Random_points_in_sphere_3<Point_3, Creator> gen(a); //radius with length random integer a
			for (int i = 0; i < no_of_pnts ; i++, ++gen){
				points.push_back(*gen);
			}
			if( no_of_pnts<=3 ){
				std::cerr << "Need more points." << std::endl;
				return -1;
			}
		}
	}
	if( !no_of_pnts ){
		char filename[200];
		std::cout << "Will be reading points from input..." << std::endl;

		Iiter_p3 eos; 				//end-of-stream iterator
		Iiter_p3 in( std::cin );	//file interator
		while( in!=eos ){
			points.push_back(*in);
			++in;
		}
	}

	Convex_hull_3 CH(3);
	Polyhedron_3 P;
	std::clock_t start;
    double duration;

	for (Points_3::const_iterator i = points.begin(); i != points.end(); ++i){
    	CH.insert(*i);
	}
    assert(CH.is_valid());

    start = std::clock();
	CGAL::convex_hull_d_to_polyhedron_3(CH,P);
	duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
	std::cout << "Duration: " << std::endl << duration << std::endl;
	
	Polyhedron_3::Vertex_iterator vi;
	Polyhedron_3::Facet_iterator fi;
	Polyhedron_3::Halfedge_around_facet_circulator circ;

	std::cout << "Vertices: " << std::endl;
	for (vi = P.vertices_begin(); vi!=P.vertices_end(); ++vi){
		std::cout << vi->point() << std::endl;
	}
	std::cout << "Facets(Vertices separated by commas): " << std::endl;
	for (fi = P.facets_begin(); fi!=P.facets_end(); ++fi){
		circ = fi->facet_begin();
		do {
			std::cout << circ->vertex()->point() << " , ";
		} while( ++circ != fi->facet_begin() );
		std::cout << std::endl;
	}

	return 0;
}



