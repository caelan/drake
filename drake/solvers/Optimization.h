#ifndef DRAKE_OPTIMIZATION_H
#define DRAKE_OPTIMIZATION_H

#include "Core.h"
#include <list>
#include <memory>
#include <initializer_list>

namespace Drake {

  // Quick notes about this design:
  // need to have lists of constraints (if not functions), so I want a (non-templated) base class.
  // would be nice to have templated types for specialized functions.  Not strictly required.
  // approach: let functions be templated, and use lambda closures/std::function to wrap them into the constraint classes.

  /**
   * @defgroup function_concept Function Concept
   * @ingroup concepts
   * @{
   * @brief Describes a mapping from one Vector to another Vector y = f(x)
   *
   * @nbsp
   *
   * | Every model of this concept must implement |  |
   * ---------------------|------------------------------------------------------------|
   * | X::InputVector     | type for the input to the system, which models the Vector<ScalarType> concept |
   * | X::OutputVector    | type for the output from the system, which models the Vector<ScalarType> concept |
   * | template <ScalarType> OutputVector<ScalarType> operator()(const InputVector<ScalarType>& x) | the actual function |
   * | InputOutputRelation getProperties() | |
   * | size_t getNumInputs() | only required if the input vector is dynamically-sized |
   * | size_t getNumOutputs() | only required if the output vector is dynamically-sized |
   *
   * (always try to label your methods with const if possible)
   *
   * @}
   */

  /*
   * some thoughts: a constraint is a function + lower and upper bounds.  it should support evaluating the constraint, adding it to an optimization problem,
   * and have support for constraints that require slack variables (adding additional decision variables to the problem).  There
   * should also be some notion of parameterized constraints:  e.g. the acceleration constraints in the rigid body dynamics are constraints
   * on vdot and f, but are "parameterized" by q and v.
   */
  /*
  class Constraint {
  public:
    Constraint(const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& func, const Eigen::MatrixBase<DerivedA>& lb, const Eigen::MatrixBase<DerivedB>& ub)
      : eval(func), lower_bound(lb), upper_bound(ub)
    {};

    template <typename FunctionType, typename DerivedA, typename DerivedB>
    Constraint(const FunctionType& func, const Eigen::MatrixBase<DerivedA>& lb, const Eigen::MatrixBase<DerivedB>& ub)
      : eval([=] (const Eigen::VectorXd& x) {return func(x);}),
        lower_bound(lb), upper_bound(ub)
    {};

    isEqualityConstraint(void) { return (upper_bound-lower_bound).isZero(); }

    std::function<Eigen::VectorXd(const Eigen::VectorXd&)> eval;
//    InputOutputRelation structure;

    Eigen::VectorXd lower_bound, upper_bound;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  // DifferentiableConstraint, PolynomialConstraint, QuadraticConstraint, ComplementarityConstraint, IntegerConstraint, ...
*/

  /** LinearConstraint
   * @brief Implements a constraint of the form @f lb \le Ax \le ub @f
   */
  class LinearConstraint { // : public Constraint {
  public:
    template <typename DerivedA,typename DerivedLB,typename DerivedUB>
    LinearConstraint(const Eigen::MatrixBase<DerivedA>& A,const Eigen::MatrixBase<DerivedLB>& lower_bound, const Eigen::MatrixBase<DerivedUB>& upper_bound)
            : A(A), lb(lower_bound), ub(upper_bound) {
      assert(lb.rows()==ub.rows());
      assert(A.rows()==lb.rows());
    }
//    template <typename FunctionType> LinearConstraint(lb,ub);  // construct using autodiff

    bool isEqualityConstraint(void) { return (ub-lb).isZero(); }

    const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> & getMatrix() const { return A; }
    const Eigen::Matrix<double,Eigen::Dynamic,1>& getLowerBound() const { return lb; }
    const Eigen::Matrix<double,Eigen::Dynamic,1>& getUpperBound() const { return ub; }

  private:
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>  A;
    Eigen::Matrix<double,Eigen::Dynamic,1> lb, ub;
  };


  class OptimizationProblem {
  public:
    OptimizationProblem() : problem_type(new LeastSquares), num_vars(0) {};

    struct DecisionVariable {
      enum VarType {
        CONTINUOUS,
        INTEGER,
        BINARY
      };
      VarType type;
      std::string name;
      Eigen::VectorXd value;
      size_t start_index;
    };
    class DecisionVariableView {  // enables users to access pieces of the decision variables like they would any other eigen vector
    public:
      DecisionVariableView(const DecisionVariable& var) : var(var), start_index(0), length(var.value.rows()) {}
      DecisionVariableView(const DecisionVariable& var, size_t start, size_t n) : var(var), start_index(start), length(n) {
        assert(start+n<var.value.rows());
      }

      size_t index() const { return var.start_index + start_index; }
      Eigen::VectorBlock<const Eigen::VectorXd,-1> value() const { return var.value.segment(start_index,length); }

      const DecisionVariableView operator()(size_t i) const { assert(i<=length); return DecisionVariableView(var,start_index+i,1);}
      const DecisionVariableView row(size_t i) const { assert(i<=length); return DecisionVariableView(var,start_index+i,1); }
      const DecisionVariableView head(size_t n) const { assert(n<=length); return DecisionVariableView(var,start_index,n); }
      const DecisionVariableView tail(size_t n) const { assert(n<=length); return DecisionVariableView(var,start_index+length-n,n); }
      const DecisionVariableView segment(size_t start, size_t n) const { assert(start+n<=length); return DecisionVariableView(var,start_index+start,n); }

      friend class OptimizationProblem;

    private:
      const DecisionVariable& var;
      size_t start_index, length;
    };

    const DecisionVariableView addContinuousVariables(std::size_t num_new_vars, std::string name = "x") {
      DecisionVariable v;
      v.type = DecisionVariable::CONTINUOUS;
      v.name = name;
      v.value = Eigen::VectorXd::Zero(num_new_vars);
      v.start_index = num_vars;
      variables.push_back(v);
      num_vars += num_new_vars;

      return DecisionVariableView(variables.back());
    }
//    const DecisionVariable& addIntegerVariables(size_t num_new_vars, std::string name);
//  ...

    typedef std::list<const DecisionVariableView> VariableList;

    // todo: consider putting the variable list into the constraint.  otherwise it's possible to update the constraint without updating (or even checking) the variable list
    template <typename ConstraintType>
    struct ConstraintContainer {
      ConstraintContainer(const std::shared_ptr<ConstraintType>& con) : constraint(con) {};
      ConstraintContainer(const std::shared_ptr<ConstraintType>& con, const std::list<const DecisionVariableView> vars) : constraint(con), variable_list(vars) {};
      std::shared_ptr<ConstraintType> constraint;
      VariableList variable_list;
    };

    /** addConstraint
     * @brief adds a constraint to the program.  method specializations ensure that the constraint gets added in the right way
     */
//    void addConstraint(const std::shared_ptr<LinearConstraint>& con, const VariableList& vars) {}

    /** addLinearEqualityConstraint
     * @brief adds linear equality constraints to the program for all (currently existing) variables
     */
    template <typename DerivedA,typename DerivedB>
    void addLinearEqualityConstraint(const Eigen::MatrixBase<DerivedA>& Aeq,const Eigen::MatrixBase<DerivedB>& beq) {
      assert(Aeq.cols() == num_vars);
      ConstraintContainer<LinearConstraint> c(std::make_shared<LinearConstraint>(Aeq,beq,beq));
      for (auto const& v : variables) {
        c.variable_list.push_back(DecisionVariableView(v));
      }

      problem_type.reset(problem_type->addLinearEqualityConstraint());
      linear_equality_constraints.push_back(c);
    }

  /** addLinearEqualityConstraint
   * @brief adds linear equality constraints referencing potentially a subset of the decision variables.
   * Example: to add and equality constraint which only depends on two of the elements of x, you could use
   *   auto x = prog.addContinuousDecisionVariable(6,"myvar");
   *   prog.addLinearEqualityConstraint(Aeq,beq,{x.row(2),x.row(5)});
   * where Aeq has exactly two columns.
   */
    template <typename DerivedA,typename DerivedB>
    void addLinearEqualityConstraint(const Eigen::MatrixBase<DerivedA>& Aeq,const Eigen::MatrixBase<DerivedB>& beq, const VariableList& vars) {
      ConstraintContainer<LinearConstraint> c(std::make_shared<LinearConstraint>(Aeq,beq,beq),vars);

      // todo: only do this loop if debug mode (only used for assert)
      int num_referenced_vars = 0;
      for (const DecisionVariableView v : vars) {
        num_referenced_vars += v.length;
      }
      assert(Aeq.cols() == num_referenced_vars);

      problem_type.reset(problem_type->addLinearEqualityConstraint());
      linear_equality_constraints.push_back(c);
    }

    // template <typename FunctionType>
    // void addCost(std::function..);
    // void addLinearCost(const Eigen::MatrixBase<Derived>& c,const vector<const DecisionVariable&>& vars)
    // void addQuadraticCost ...

//    void addConstraint(const LinearConstraint& constraint, const std::vector<const DecisionVariable&>& vars);
//  void addConstraint(const BoundingBoxConstraint& constraint, const std::vector<const DecisionVariable&>& vars);

//    template <typename DerivedLB,typename DerivedUB>
//    void addBoundingBoxConstraint(const Eigen::MatrixBase<DerivedLB>& lower_bound, const Eigen::MatrixBase<DerivedUB>& upper_bound, const DecisionVariable& var) {  }


    bool solve() { return problem_type->solve(*this); }; // todo: add argument for options

//    template <typename Derived>
//    bool solve(const Eigen::MatrixBase<Derived>& x0);

//    getObjectiveValue();
//    getExitFlag();
//    getInfeasibleConstraintNames();

    void printSolution() {  // todo: overload the ostream operator instead?
      for (const auto& v : variables) {
        std::cout << v.name << " = " << v.value.transpose() << std::endl;
      }
    }


  private:
    template <typename Derived>
    void setSolution(const Eigen::MatrixBase<Derived>& x) {
      size_t index=0;
      for (auto &v : variables) {
        v.value = x.middleRows(index,v.value.rows());
        index += v.value.rows();
      }
    }

    // note: use std::list instead of std::vector because realloc in std::vector invalidates existing references to the elements
    std::list<DecisionVariable> variables;
    std::list<ConstraintContainer<LinearConstraint>> linear_equality_constraints;
    size_t num_vars;

  private:
    // uses virtual methods to crawl up the complexity hiearchy as new decision variables and constraints are added to the program
    // note that there is dynamic allocation happening in here, but on a structure of negligible size.  (is there a better way?)
    struct MathematicalProgram {
    /* these would be used to fill out the optimization hierarchy prototyped below
      virtual MathematicalProgram* addIntegerVariable() { return new MathematicalProgram; };

      virtual MathematicalProgram* addLinearCost() { return new MathematicalProgram; };
      virtual MathematicalProgram* addQuadraticCost() { return new MathematicalProgram; };
      virtual MathematicalProgram* addNonlinearCost() { return new MathematicalProgram; };

      virtual MathematicalProgram* addSumsOfSquaresConstraint() { return new MathematicalProgram; };
      virtual MathematicalProgram* addLinearMatrixInequalityConstraint() { return new MathematicalProgram; };
      virtual MathematicalProgram* addSecondOrderConeConstraint() { return new MathematicalProgram; };
      virtual MathematicalProgram* addComplementarityConstraint() { return new MathematicalProgram; };
      virtual MathematicalProgram* addNonlinearConstraint() { return new MathematicalProgram; };
      virtual MathematicalProgram* addLinearInequalityConstraint() { return new MathematicalProgram; };
     */
      virtual MathematicalProgram* addLinearEqualityConstraint() { return new MathematicalProgram; };

      virtual bool solve(OptimizationProblem& prog) { throw std::runtime_error("not implemented yet"); }
    };

/*  // Prototype of the more complete optimization problem class hiearchy (to be implemented only as needed)
    struct MixedIntegerNonlinearProgram : public MathematicalProgram {};
    struct MixedIntegerSemidefiniteProgram : public MixedIntegerNonlinearProgram {};
    struct MixedIntegerSecondOrderConeProgram : public MixedIntegerSemidefiniteProgram {};
    struct MixedIntegerQuadraticProgram : public MixedIntegerSecondOrderConeProgram {};
    struct MixedIntegerLinearProgram : public MixedIntegerQuadraticProgram {};

    struct NonlinearProgram : public MixedIntegerNonlinearProgram {};
    struct SemidefiniteProgram : public NonlinearProgram, public MixedIntegerSemidefiniteProgram {};
    struct SecondOrderConeProgram : public SemidefiniteProgram, public MixedIntegerSecondOrderConeProgram {};
    struct QuadraticProgram : public SecondOrderConeProgram, public MixedIntegerQuadraticProgram {};
    struct LinearProgram : public QuadraticProgram, public MixedIntegerLinearProgram {
      virtual MathematicalProgram* addLinearEqualityConstraint() { return new LinearProgram; };
      virtual MathematicalProgram* addLinearInequalityConstraint() { return new LinearProgram; };
    };

    struct NonlinearComplementarityProblem : public NonlinearProgram {};
    struct LinearComplementarityProblem : public NonlinearComplementarityProblem {};
*/
    struct LeastSquares : public MathematicalProgram { //public LinearProgram, public LinearComplementarityProblem {
      virtual MathematicalProgram* addLinearEqualityConstraint() { return new LeastSquares; };
      virtual bool solve(OptimizationProblem& prog) {
        size_t num_constraints = 0;
        for (auto const& cc : prog.linear_equality_constraints) {
          num_constraints += cc.constraint->getMatrix().rows();
        }

        Eigen::MatrixXd Aeq = Eigen::MatrixXd::Zero(num_constraints,prog.num_vars);  // todo: use a sparse matrix here?
        Eigen::VectorXd beq(num_constraints);

        size_t constraint_index = 0;
        for (auto const& cc : prog.linear_equality_constraints) {
          const LinearConstraint& con = *cc.constraint;
          for (const DecisionVariableView& v : cc.variable_list ) {
            size_t n = con.getMatrix().rows();
            Aeq.block(constraint_index, v.index(), n, v.length) = con.getMatrix();
            beq.segment(constraint_index, n) = con.getLowerBound();  // = con.getUpperBound() since it's an equality constraint
            constraint_index += n;
          }
        }

        // least-squares solution
        prog.setSolution(Aeq.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(beq));
        return true;
      }
    };
    std::shared_ptr<MathematicalProgram> problem_type;
  };

} // end namespace Drake

#endif //DRAKE_OPTIMIZATION_H
