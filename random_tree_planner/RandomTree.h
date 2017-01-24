#include <vector>
#include <functional>
#include <ompl/base/StateSpace.h>

namespace ompl
{
    /** \brief Abstract representation of a container that can perform nearest neighbors queries */
    template <typename _T>
    class TreeStruct
    {
    public:
        /** \brief The definition of a distance function */
        typedef std::function<double(const _T &, const _T &)> DistanceFunction;

        TreeStruct() = default;

        virtual ~TreeStruct() = default;

        /** \brief Set the distance function to use */
        virtual void setDistanceFunction(const DistanceFunction &distFun)
        {
            distFun_ = distFun;
        }

        /** \brief Get the distance function used */
        const DistanceFunction &getDistanceFunction() const
        {
            return distFun_;
        }

        /** \brief Return true if the solutions reported by this data structure
            are sorted, when calling nearestK / nearestR. */
        virtual bool reportsSortedResults() const = 0;

        /** \brief Clear the datastructure */
        virtual void clear() = 0;

        /** \brief Add an element to the datastructure */
        virtual void add(const _T &data) = 0;

        /** \brief Add a vector of points */
        virtual void add(const std::vector<_T> &data)
        {
            for (auto elt = data.begin(); elt != data.end(); ++elt)
                add(*elt);
        }

        /** \brief Remove an element from the datastructure */
        virtual bool remove(const _T &data) = 0;

        /** \brief Get the nearest neighbor of a point */
        virtual _T nearest(const _T &data) const = 0;

        /** \brief Get the k-nearest neighbors of a point
         *
         * All the nearest neighbor structures currently return the neighbors in
         * sorted order, but this is not required.
         */

        /** \brief Get the number of elements in the datastructure */
        virtual std::size_t size() const = 0;

        /** \brief Get all the elements in the datastructure */
        virtual void list(std::vector<_T> &data) const = 0;

    protected:
        /** \brief The used distance function */
        DistanceFunction distFun_;
    };
}