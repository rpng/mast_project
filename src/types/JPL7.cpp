
#include "types/JPL7.h"
#include <g2o/core/factory.h>
#include <g2o/stuff/macros.h>

using namespace MAST;
JPL7::JPL7() : g2o::BaseVertex<6, Eigen::Matrix<double,7,1> >()
{
    _marginalized=false;
}

bool JPL7::write(std::ostream& os) const
{
	// TODO
	assert(false);
	return false;
}

bool JPL7::read(std::istream& is)
{
	// TODO
	assert(false);
	return false;
}


Feature::Feature() : g2o::BaseVertex<3, Eigen::Matrix<double,3,1> >()
{
    _marginalized=false;
}

bool Feature::write(std::ostream& os) const
{
	// TODO
	assert(false);
	return false;
}

bool Feature::read(std::istream& is)
{
	// TODO
	assert(false);
	return false;
}

ImageEdge::ImageEdge() : g2o::BaseBinaryEdge<2, Eigen::Matrix<double, 2,1>, JPL7, Feature>(){

    }


bool ImageEdge::write(std::ostream& os) const
{
	// TODO
	assert(false);
	return false;

}

bool ImageEdge::read(std::istream& is)
{
	// TODO
	assert(false);
	return false;
//     Vector7d v7;
}


SonarEdge::SonarEdge() :g2o::BaseMultiEdge<2,Eigen::Matrix<double,2,1> >(){
    }

bool SonarEdge::write(std::ostream& os) const
{
	// TODO
	assert(false);
	return false;

}

bool SonarEdge::read(std::istream& is)
{
	// TODO
	assert(false);
	return false;
//     Vector7d v7;
}

PriorEdge::PriorEdge() : g2o::BaseUnaryEdge<6, Eigen::Matrix<double,6,1>, JPL7 >(){

}

bool PriorEdge::write(std::ostream& os) const
{
	// TODO
	assert(false);
	return false;

}

bool PriorEdge::read(std::istream& is)
{
	// TODO
	assert(false);
	return false;
//     Vector7d v7;
}