//! Definitions for decoding parameter expressions from pytket operations.
//!
//! This is based on the `pest` grammar defined in `param.pest`.

use cgmath::RelativeEq;
use derive_more::Display;
use hugr::ops::OpType;
use hugr::std_extensions::arithmetic::float_ops::FloatOps;
use itertools::Itertools;
use pest::Parser;
use pest::iterators::{Pair, Pairs};
use pest::pratt_parser::PrattParser;
use pest_derive::Parser;

use crate::extension::rotation::RotationOp;
use crate::serialize::pytket::decoder::ParameterType;

/// The parsed AST for a pytket operation parameter.
///
/// The leafs of the AST are either a constant value, a variable name, or an
/// unrecognized sympy expression.
#[derive(Debug, Display, Clone)]
pub enum PytketParam<'a> {
    /// A constant value that can be loaded directly.
    #[display("{_0}")]
    Constant(f64),
    /// A variable that should be routed as an input.
    #[display("\"{name}\"")]
    InputVariable {
        /// The variable name.
        name: &'a str,
    },
    /// Unrecognized sympy expression.
    /// Will be emitted as a [`SympyOp`].
    #[display("Sympy(\"{_0}\")")]
    Sympy(&'a str),
    /// An operation on some nested expressions.
    #[display("{}({})", op.to_string(), args.iter().map(|a| a.to_string()).join(", "))]
    Operation {
        /// The HUGR operation used to implement this node.
        op: OpType,
        /// Input arguments to the operation.
        args: Vec<PytketParam<'a>>,
        /// The parameter types used for the inputs and outputs of this operation.
        param_ty: ParameterType,
    },
}

impl<'a> PytketParam<'a> {
    /// Parse a TKET1 operation parameter, and return an AST representing the expression.
    #[inline]
    pub fn parse(param: &'a str) -> Self {
        let Ok(mut parsed) = ParamParser::parse(Rule::parameter, param) else {
            // The parameter could not be parsed, so we just return it as an opaque sympy expression.
            return PytketParam::Sympy(param);
        };
        let parsed = parsed
            .next()
            .expect("The `parameter` rule can only be matched once.");

        parse_infix_ops(parsed.into_inner())
    }

    /// Returns `true` if the parameter is zero.
    pub fn is_zero(&self) -> bool {
        matches!(self, PytketParam::Constant(value) if *value == 0.0)
    }

    /// Visit the input variables referenced by this parsed expression.
    ///
    /// Variables inside unrecognized SymPy expressions cannot be routed by the
    /// decoder, so [`Self::Sympy`] is treated as an opaque leaf.
    pub fn visit_input_variables(&self, visitor: &mut impl FnMut(&'a str)) {
        match self {
            Self::InputVariable { name } => visitor(name),
            Self::Operation { args, .. } => {
                for arg in args {
                    arg.visit_input_variables(visitor);
                }
            }
            Self::Constant(_) | Self::Sympy(_) => {}
        }
    }
}

impl<'a> PartialEq for PytketParam<'a> {
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (Self::Constant(l0), Self::Constant(r0)) => {
                l0.relative_eq(r0, f64::EPSILON, f64::EPSILON)
            }
            (Self::InputVariable { name: l_name }, Self::InputVariable { name: r_name }) => {
                l_name == r_name
            }
            (Self::Sympy(l0), Self::Sympy(r0)) => l0 == r0,
            (
                Self::Operation {
                    op: l_op,
                    args: l_args,
                    param_ty: l_param_ty,
                },
                Self::Operation {
                    op: r_op,
                    args: r_args,
                    param_ty: r_param_ty,
                },
            ) => l_op == r_op && l_args == r_args && l_param_ty == r_param_ty,
            _ => false,
        }
    }
}

impl<'a> Eq for PytketParam<'a> {}

impl<'a> std::hash::Hash for PytketParam<'a> {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        match self {
            Self::InputVariable { name } => name.hash(state),
            Self::Sympy(expr) => expr.hash(state),
            _ => self.to_string().hash(state),
        }
    }
}

#[derive(Parser)]
#[grammar = "serialize/pytket/decoder/param/param.pest"]
struct ParamParser;

lazy_static::lazy_static! {
    /// Precedence parser used to define the order of infix operations.
    ///
    /// Based on the calculator example from `pest`.
    /// https://pest.rs/book/examples/calculator.html
    static ref PRATT_PARSER: PrattParser<Rule> = {
        use pest::pratt_parser::{Assoc::*, Op};
        use Rule::*;

        // Precedence is defined lowest to highest
        PrattParser::new()
            // Addition and subtract have equal precedence
            .op(Op::infix(add, Left) | Op::infix(subtract, Left))
            .op(Op::infix(multiply, Left) | Op::infix(divide, Left))
            .op(Op::infix(power, Left))
    };
}

/// Parse a match of the [`Rule::expr`] rule.
///
/// This takes a sequence of rule matches alternating [`Rule::term`]s and infix operations.
fn parse_infix_ops(pairs: Pairs<'_, Rule>) -> PytketParam<'_> {
    use ParameterType::*;

    PRATT_PARSER
        .map_primary(|primary| parse_term(primary))
        .map_infix(|lhs, op, rhs| {
            let (op, param_ty) = match op.as_rule() {
                Rule::add => (RotationOp::radd.into(), Rotation),
                Rule::subtract => (FloatOps::fsub.into(), FloatHalfTurns),
                Rule::multiply => (FloatOps::fmul.into(), FloatHalfTurns),
                Rule::divide => (FloatOps::fdiv.into(), FloatHalfTurns),
                Rule::power => (FloatOps::fpow.into(), FloatHalfTurns),
                rule => unreachable!("Expr::parse expected infix operation, found {:?}", rule),
            };
            PytketParam::Operation {
                op,
                args: vec![lhs, rhs],
                param_ty,
            }
        })
        .parse(pairs)
}

/// Parse a match of the silent [`Rule::term`] rule.
fn parse_term(pair: Pair<'_, Rule>) -> PytketParam<'_> {
    use ParameterType::*;

    match pair.as_rule() {
        Rule::expr => parse_infix_ops(pair.into_inner()),
        Rule::implicit_multiply => {
            let mut pairs = pair.into_inner();
            let lhs = parse_term(pairs.next().unwrap());
            let rhs = parse_term(pairs.next().unwrap());
            PytketParam::Operation {
                op: FloatOps::fmul.into(),
                args: vec![lhs, rhs],
                param_ty: FloatHalfTurns,
            }
        }
        Rule::num => parse_number(pair),
        Rule::unary_minus => PytketParam::Operation {
            op: FloatOps::fneg.into(),
            args: vec![parse_term(pair.into_inner().next().unwrap())],
            param_ty: FloatHalfTurns,
        },
        Rule::function_call => parse_function_call(pair),
        Rule::ident => PytketParam::InputVariable {
            name: pair.as_str(),
        },
        rule => unreachable!("Term::parse expected a term, found {:?}", rule),
    }
}

/// Parse a match of the [`Rule::num`] rule.
fn parse_number(pair: Pair<'_, Rule>) -> PytketParam<'_> {
    let num = pair.as_str();
    let half_turns = num
        .parse::<f64>()
        .unwrap_or_else(|_| panic!("`num` rule matched invalid number \"{num}\""));
    PytketParam::Constant(half_turns)
}

/// Parse a match of the [`Rule::function_call`] rule.
fn parse_function_call(pair: Pair<'_, Rule>) -> PytketParam<'_> {
    let pair_str = pair.as_str();
    let mut args = pair.into_inner();
    let name = args
        .next()
        .expect("Function call must have a name")
        .as_str();
    let op = match name {
        "max" => FloatOps::fmax.into(),
        "min" => FloatOps::fmin.into(),
        "abs" => FloatOps::fabs.into(),
        "floor" => FloatOps::ffloor.into(),
        "ceil" => FloatOps::fceil.into(),
        "round" => FloatOps::fround.into(),
        // Unrecognized function name.
        // Treat it as an opaque sympy expression.
        _ => return PytketParam::Sympy(pair_str),
    };

    let args = args.map(|arg| parse_term(arg)).collect::<Vec<_>>();
    PytketParam::Operation {
        op,
        args,
        param_ty: ParameterType::FloatHalfTurns,
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use rstest::rstest;

    #[rstest]
    #[case::int("42", PytketParam::Constant(42.0))]
    #[case::float("42.37", PytketParam::Constant(42.37))]
    #[case::float_pointless("37.", PytketParam::Constant(37.))]
    #[case::exp("42e4", PytketParam::Constant(42e4))]
    #[case::neg("-42.55", PytketParam::Constant(-42.55))]
    #[case::parens("(42)", PytketParam::Constant(42.))]
    #[case::var("f64", PytketParam::InputVariable{name: "f64"})]
    #[case::add("42 + f64", PytketParam::Operation {
        op: RotationOp::radd.into(),
        args: vec![PytketParam::Constant(42.), PytketParam::InputVariable{name: "f64"}],
        param_ty: ParameterType::Rotation,
    })]
    #[case::sub("42 - 2", PytketParam::Operation {
        op: FloatOps::fsub.into(),
        args: vec![PytketParam::Constant(42.), PytketParam::Constant(2.)],
        param_ty: ParameterType::FloatHalfTurns,
    })]
    #[case::product_implicit("42 f64", PytketParam::Operation {
        op: FloatOps::fmul.into(),
        args: vec![PytketParam::Constant(42.), PytketParam::InputVariable{name: "f64"}],
        param_ty: ParameterType::FloatHalfTurns,
    })]
    #[case::product_implicit2("42f64", PytketParam::Operation {
        op: FloatOps::fmul.into(),
        args: vec![PytketParam::Constant(42.), PytketParam::InputVariable{name: "f64"}],
        param_ty: ParameterType::FloatHalfTurns,
    })]
    #[case::product_implicit3("42 e4", PytketParam::Operation {
        op: FloatOps::fmul.into(),
        args: vec![PytketParam::Constant(42.), PytketParam::InputVariable{name: "e4"}],
        param_ty: ParameterType::FloatHalfTurns,
    })]
    #[case::max("max(42, f64)", PytketParam::Operation {
        op: FloatOps::fmax.into(),
        args: vec![PytketParam::Constant(42.), PytketParam::InputVariable{name: "f64"}],
        param_ty: ParameterType::FloatHalfTurns,
    })]
    #[case::minus("-f64", PytketParam::Operation {
        op: FloatOps::fneg.into(),
        args: vec![PytketParam::InputVariable{name: "f64"}],
        param_ty: ParameterType::FloatHalfTurns,
    })]
    #[case::unknown("unknown_op(42, f64)", PytketParam::Sympy("unknown_op(42, f64)"))]
    #[case::unknown_no_params("unknown_op()", PytketParam::Sympy("unknown_op()"))]
    #[case::nested("max(42, unknown_op(37))", PytketParam::Operation {
        op: FloatOps::fmax.into(),
        args: vec![PytketParam::Constant(42.), PytketParam::Sympy("unknown_op(37)")],
        param_ty: ParameterType::FloatHalfTurns,
    })]
    #[case::precedence("5-2/3x+4**6", PytketParam::Operation {
        op: RotationOp::radd.into(),
        args: vec![
            PytketParam::Operation {
                op: FloatOps::fsub.into(),
                args: vec![
                    PytketParam::Constant(5.),
                    PytketParam::Operation {
                        op: FloatOps::fdiv.into(),
                        args: vec![
                            PytketParam::Constant(2.),
                            PytketParam::Operation {
                                op: FloatOps::fmul.into(),
                                args: vec![
                                    PytketParam::Constant(3.),
                                    PytketParam::InputVariable{name: "x"},
                                ],
                                param_ty: ParameterType::FloatHalfTurns,
                            },
                        ],
                        param_ty: ParameterType::FloatHalfTurns,
                    },
                ],
                param_ty: ParameterType::FloatHalfTurns,
            },
            PytketParam::Operation {
                op: FloatOps::fpow.into(),
                args: vec![PytketParam::Constant(4.), PytketParam::Constant(6.)],
                param_ty: ParameterType::FloatHalfTurns,
            },
        ],
        param_ty: ParameterType::Rotation,
    })]
    #[case::associativity("1-2-3+4", PytketParam::Operation {
        op: RotationOp::radd.into(),
        args: vec![
            PytketParam::Operation {
                op: FloatOps::fsub.into(),
                args: vec![
                    PytketParam::Operation {
                        op: FloatOps::fsub.into(),
                        args: vec![PytketParam::Constant(1.), PytketParam::Constant(2.)],
                        param_ty: ParameterType::FloatHalfTurns,
                    },
                    PytketParam::Constant(3.),
                ],
                param_ty: ParameterType::FloatHalfTurns,
            },
            PytketParam::Constant(4.),
        ],
        param_ty: ParameterType::Rotation,
    })]
    fn parse_param(#[case] param: &str, #[case] expected: PytketParam) {
        let parsed = PytketParam::parse(param);
        if parsed != expected {
            panic!(
                "Incorrect parameter parsing\n\texpression: \"{param}\"\n\tparsed: {parsed}\n\texpected: {expected}"
            );
        }
    }

    #[rstest]
    #[case::constant("42", &[])]
    #[case::variable("p0", &["p0"])]
    #[case::nested("2 * (p0 + p1 + pi)", &["p0", "p1", "pi"])]
    #[case::opaque_sympy("unknown_op(p0)", &[])]
    fn visit_input_variables(#[case] param: &str, #[case] expected: &[&str]) {
        let parsed = PytketParam::parse(param);
        let mut variables = Vec::new();
        parsed.visit_input_variables(&mut |name| variables.push(name));
        assert_eq!(variables, expected);
    }
}
