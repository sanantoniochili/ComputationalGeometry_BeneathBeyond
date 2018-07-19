#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/point_generators_3.h>
#include <CGAL/algorithm.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/enum.h> 

#include <CGAL/IO/Geomview_stream.h>
#include <CGAL/IO/Polyhedron_geomview_ostream.h>

#include <iterator>
#include <iostream>
#include <fstream>
#include <cassert>
#include <string>
#include <cstdio>
#include <vector>
#include <ctime>
#include <list>

typedef CGAL::Exact_predicates_exact_constructions_kernel K;
typedef CGAL::Polyhedron_3< K> 								Polyhedron_3;
typedef K::Point_3 											Point_3;
typedef CGAL::Plane_3<K>									Plane_3;
typedef CGAL::Creator_uniform_3<double,Point_3>  			Creator;
typedef std::istream_iterator<Point_3>  					Iiter_p3;
typedef std::vector<Point_3> 								Points_3;
typedef Polyhedron_3::Facet 								Facet;
typedef Polyhedron_3::HalfedgeDS     				        HalfedgeDS;
typedef CGAL::Polyhedron_incremental_builder_3<HalfedgeDS>	Builder;

typedef HalfedgeDS::Face_iterator							Face_iterator;
typedef HalfedgeDS::Halfedge_iterator						Halfedge_iterator;
typedef HalfedgeDS::Vertex_iterator							Vertex_iterator;
typedef HalfedgeDS::Face_handle								Face_handle;
typedef HalfedgeDS::Halfedge_handle							Halfedge_handle;
typedef HalfedgeDS::Vertex_handle							Vertex_handle;

bool wayToSort(Point_3 i, Point_3 j) { return i > j; }

template<class HDS>
class Build_polyhedron : public CGAL::Modifier_base<HDS> {
private:
 Points_3  &Points;
 int 	   Index;
 int 	   AbsIndex;	//absolute index of points of polyhedron
 int       Scale;
 std::map<Point_3,int> iPoints;	//keeps indices of vertices
public:
     Build_polyhedron( Points_3 &_points, int &_scale ) : Points(_points), Index(0), AbsIndex(0), Scale(_scale) {}
     void Draw( HDS& hds ) {
     	std::ofstream myfile;
     	myfile.open("output.txt");

     	CGAL::Geomview_stream gv(CGAL::Bbox_3(-2*Scale, -2*Scale, -2*Scale, 2*Scale, 2*Scale, 2*Scale));
     	gv.set_line_width( 4 );
     	gv.set_bg_color(CGAL::Color( 0,200,200 ));
     	gv << CGAL::BLUE;
     	std::cout << "Draw Vertices" << std::endl;
     	for (Vertex_iterator i = hds.vertices_begin(); i != hds.vertices_end(); ++i)
     	{
     		myfile << i->point() << std::endl;
     		gv << i->point();
     	}
     	sleep(5);
  		gv.clear();
     }
     void printHDS( HDS& hds ) {
     	int counter = 0;
     	std::cout << std::endl << "HalfedgeDS structure." << std::endl;
     	for (Face_iterator facet = hds.faces_begin(); facet != hds.faces_end(); ++facet)
     	{
     		std::cout << "Face " << counter << ":" << std::endl;
     		Halfedge_handle h = facet->halfedge();
     		std::cout << "Point 0: " << h->vertex()->point() << std::endl;
     		std::cout << "Point 1: " << h->next()->vertex()->point() << std::endl;
     		std::cout << "Point 2: " << h->next()->next()->vertex()->point() << std::endl;
     		++counter;
     	}
     	std::cout << std::endl;
     }
     void operator()( HDS& hds) {
  		typedef typename HDS::Vertex Vertex;
 
  		// create a cgal incremental builder
        Builder B( hds, true);
        B.begin_surface( 4,4,12 );
   
		  // add the tetrahedron vertices
		  for( int i=0; i<4; i++ ){
		  	B.add_vertex( Points[i] );
		  	iPoints.insert( std::make_pair( Points[i],i ) );
		  }
		  Index = 4;
		  AbsIndex = 4;
		  B.begin_facet();
		  B.add_vertex_to_facet(1);
		  B.add_vertex_to_facet(3);
		  B.add_vertex_to_facet(0);
		  B.end_facet();
		  B.begin_facet();
		  B.add_vertex_to_facet(2);
		  B.add_vertex_to_facet(1);
		  B.add_vertex_to_facet(0);
		  B.end_facet();
		  B.begin_facet();
		  B.add_vertex_to_facet(3);
		  B.add_vertex_to_facet(2);
		  B.add_vertex_to_facet(0);
		  B.end_facet();
		  B.begin_facet();
		  B.add_vertex_to_facet(2);
		  B.add_vertex_to_facet(3);
		  B.add_vertex_to_facet(1);
		  B.end_facet();
		B.end_surface();

		std::cout << "Tetrahedron" << std::endl;
		printHDS( hds );

		std::vector<Face_handle> 	 Blue;
		std::vector<Face_handle>	 Red;
		std::vector<Vertex_handle>   Blue_V;
		std::vector<Vertex_handle>   Red_V;
		std::vector<Vertex_handle>   Purple_V;

		for (; Index < Points.size(); ++Index)
		{
			Point_3 p0 = Points[Index];	//new point
			std::cout << "New point: " << p0 << std::endl;
			int onCH = 0;

			Blue.clear();
			Red.clear();
			Blue_V.clear();
			Red_V.clear();
			Purple_V.clear();

			int count = 0;
        	for (Face_iterator facet = hds.faces_begin(); facet != hds.faces_end(); ++facet)
        	{
        		//find facet plane
        		std::cout << "Face " << count << ":";
        		Halfedge_handle h = facet->halfedge();
        		Plane_3 plane = Plane_3( h->vertex()->point(),
                 					   	 h->next()->vertex()->point(),
                  					     h->next()->next()->vertex()->point());

        		//check position of point in relation to the plane 
        		CGAL::Oriented_side side_p0 = plane.oriented_side ( p0 );
        		
        		//find a point on hull that does not belong to facet 
        		Vertex_iterator temp_v = hds.vertices_begin();
        		while( temp_v!=hds.vertices_end() ){
        			if( temp_v!=h->vertex() ){
        				if( temp_v!=h->next()->vertex() ){
        					if( temp_v!=h->next()->next()->vertex() )
        						break;
        				}
        			}
        			++temp_v;
        		}
        		//check position of point in relation to the plane 
        		CGAL::Oriented_side side_p = plane.oriented_side ( temp_v->point() );

        		if( side_p0!=side_p ){	//red facet if true, blue if false
        			std::cout << "is red." << std::endl;
        			Red.push_back( Face_handle(facet) );
        			Red_V.push_back( Vertex_handle(facet->halfedge()->vertex()) );
        			Red_V.push_back( Vertex_handle(facet->halfedge()->next()->vertex()) );
        			Red_V.push_back( Vertex_handle(facet->halfedge()->next()->next()->vertex()) );
        			onCH = 1;	//must be included to CHs
        		} else {
        			std::cout << "is blue." << std::endl;
        			Blue.push_back( Face_handle(facet) );
        			Blue_V.push_back( Vertex_handle(facet->halfedge()->vertex()) );
        			Blue_V.push_back( Vertex_handle(facet->halfedge()->next()->vertex()) );
        			Blue_V.push_back( Vertex_handle(facet->halfedge()->next()->next()->vertex()) );

        		}
        		++count;
   			}

        	if( onCH==1 ){
				//Draw( hds );	//uncomment to see vertices
	        	//erase red vertices
	        	std::vector<Vertex_handle> Red_visited;
	        	for (std::vector<Vertex_handle>::iterator i = Red_V.begin(); i != Red_V.end(); ++i)
	        	{
	        		Vertex_handle red = *i;
	        		std::vector<Vertex_handle>::iterator it_R = find (Red_visited.begin(), Red_visited.end(), red);
	        		std::vector<Vertex_handle>::iterator it_B = find (Blue_V.begin(), Blue_V.end(), red);
	        		if( it_B != Blue_V.end() ){	//found in Blue_V vector, so it's purple
	        			Vertex_handle blue = *it_B;
	        			Purple_V.push_back( red );
	        		} else {	//not purple
	        			if (it_R == Red_visited.end())
	        			Red_visited.push_back( red );
	        		}
	        	}
	        	//erase red vertices
	        	for (std::vector<Vertex_handle>::iterator i = Red_visited.begin(); i != Red_visited.end(); ++i)
	        	{
	        		Vertex_handle red = *i;
	        		hds.vertices_erase( red );
	        	}
	        	//erase red faces
	        	for (std::vector<Face_handle>::iterator facet = Red.begin(); facet != Red.end(); ++facet)
	        	{
	        		Face_handle red = *facet;
	        		hds.faces_erase( red );
	        	}
	        	std::map<Point_3,int>::iterator mit;
	        	printHDS( hds );		
				//Draw( hds );

	        	Builder B( hds, true);	
	        	B.begin_surface( 1, Purple_V.size()/2, 0, Builder::ABSOLUTE_INDEXING);
	        	  B.add_vertex( p0 );
				  //Draw( hds );
	        	  iPoints.insert( std::make_pair( p0,AbsIndex++ ) );
	        	  for (std::vector<Vertex_handle>::iterator i = Purple_V.begin(); i+1 != Purple_V.end(); ++i)
	        	  {
	        	  	B.begin_facet();
	        	  	Vertex_handle vh1 = *i;
	        	  	Vertex_handle vh2 = *(i+1);	//add in reverse order than the order added to structure
	        	  	std::map<Point_3,int>::iterator mit;
	        	  	mit = iPoints.find( vh2->point() );
	        	  	B.add_vertex_to_facet( mit->second );
	        	  	mit = iPoints.find( vh1->point() );
	        	  	B.add_vertex_to_facet( mit->second );
	        	  	B.add_vertex_to_facet( hds.size_of_vertices()-1 );
	        	  	B.end_facet();
	        	  }
	        	B.end_surface();
   			} 
		}

    }
};
 
    

int main(int argc, char* argv[])
{	
	unsigned seed;
	Points_3 points;
	int no_of_pnts = 0;
	int a;

	srand(seed);
	if( argc==3 ){
		if( strcmp( argv[1],"-generate" )==0 ){
			no_of_pnts = atoi( argv[2] );
		}
		if( no_of_pnts>0 ){
			std::cout << "Will be generating random points..." << std::endl;
			a = rand()%1000;
			CGAL::Random_points_in_sphere_3<Point_3, Creator> gen(a); 
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
		a = 1;
		std::cout << "Will be reading points from input..." << std::endl;

		Iiter_p3 eos; 				//end-of-stream iterator
		Iiter_p3 in( std::cin );	//file interator
		while( in!=eos ){
			points.push_back(*in);
			++in;
		}
	}

	std::sort(points.begin(), points.end(), wayToSort);    //ascending lexicographical order  

	std::cout << "All points,sorted." << std::endl;
	for (Points_3::const_iterator i = points.begin(); i != points.end(); ++i){
    	std::cout << *i << std::endl;
	} 

	Polyhedron_3 P;
	Build_polyhedron<HalfedgeDS> builder( points,a );
	P.delegate(builder);

	return 0;
}


