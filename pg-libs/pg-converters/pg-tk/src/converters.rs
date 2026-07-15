use std::collections::HashSet;

use pg_core::{GateData, GateType, Op, Pauli, PauliGraph, RotationData, TableauData};
use serde_json::{Value, json};
use uuid::Uuid;

/// Error type for TKET conversion operations.
#[derive(Debug, thiserror::Error)]
pub enum TKConversionError {
    /// Invalid input TKET JSON.
    #[error("Invalid input TKET JSON: {0}")]
    InvalidTKJson(String),
    /// Unsupported input TKET JSON.
    #[error("Unsupported input TKET JSON: {0}")]
    UnsupportedTKJson(String),
    /// Unsupported operation during conversion.
    #[error("{0}")]
    UnsupportedPGOp(String),
}

// =============================================================================
//                   PauliGraph to TKET conversion functions
// =============================================================================

/// Convert a PG unsigned integer index to a TKET qubit.
fn tk_qb(q: usize) -> Value {
    json!(["q", [q]])
}

/// Convert a PG unsigned integer index to a TKET bit.
fn tk_b(b: usize) -> Value {
    json!(["c", [b]])
}

/// Convert a slice of PG unsigned integer indices to a vector of TKET qubits.
fn tk_qbs(qbs: &[usize]) -> Vec<Value> {
    qbs.iter().map(|q| json!(["q", [q]])).collect()
}

/// Convert a slice of PG unsigned integer indices to a vector of TKET bits.
fn tk_bs(bs: &[usize]) -> Vec<Value> {
    bs.iter().map(|b| json!(["c", [b]])).collect()
}

/// Convert a gate type and its parameters to a TKET gate.
fn tk_gate_json(t: &str, params: &[f64]) -> Value {
    json!({ "type": t, "params": params.iter().map(f64::to_string).collect::<Vec<_>>() })
}

/// Construct a TKET Box.
fn tk_box_json(t: &str, id: &str, payload: Value) -> Value {
    let mut box_json = json!({
        "id": id,
        "type": t,
    });
    for (key, value) in payload.as_object().unwrap() {
        box_json[key] = value.clone();
    }
    json!({
        "box": box_json,
        "type": t
    })
}

/// Construct a TKET command.
fn tk_cmd_json(args: &[Value], op_json: Value) -> Value {
    json!({
        "op": op_json,
        "args": args,
    })
}

/// Construct a TKET conditional operation.
fn tk_conditional_json(op_json: Value, cond_bits: &[Value], cond_values: &[bool]) -> Value {
    json!({
        "conditional":{
            "op": op_json,
            "width": cond_bits.len(),
            // little-endian encoding of the condition values
            "values": cond_values.iter().enumerate().fold(0u64, |acc, (i, &b)| acc | ((b as u64) << i)),
        },
        "type": "Conditional",
    })
}

/// Helper function to construct a TKET gate command with optional conditional bits and values.
fn tk_gate_cmd_json(
    t: &str,
    params: &[f64],
    args: &[Value],
    cond_bits: &[Value],
    cond_values: &[bool],
) -> Value {
    let gate_json = tk_gate_json(t, params);
    if cond_bits.is_empty() {
        tk_cmd_json(args, gate_json)
    } else {
        tk_cmd_json(
            &[cond_bits, args].concat(),
            tk_conditional_json(gate_json, cond_bits, cond_values),
        )
    }
}

/// Convert a `GateData` to a vector of TKET commands.
fn gate_to_tk(gate_data: &GateData) -> Result<Vec<Value>, TKConversionError> {
    let args = gate_data.get_args();
    let params = gate_data.get_params();
    let cond_bits = tk_bs(gate_data.get_conditional_bits());
    let cond_values = gate_data.get_conditional_values();

    let g = |t: &str, p: &[f64], a: Vec<Value>| tk_gate_cmd_json(t, p, &a, &cond_bits, cond_values);

    let gate_json = match gate_data.get_gate_type() {
        GateType::X => vec![g("X", &[], tk_qbs(args))],
        GateType::Y => vec![g("Y", &[], tk_qbs(args))],
        GateType::Z => vec![g("Z", &[], tk_qbs(args))],

        GateType::H => vec![g("H", &[], tk_qbs(args))],
        GateType::S => vec![g("S", &[], tk_qbs(args))],
        GateType::Sdg => vec![g("Sdg", &[], tk_qbs(args))],
        GateType::V => vec![g("V", &[], tk_qbs(args))],
        GateType::Vdg => vec![g("Vdg", &[], tk_qbs(args))],

        GateType::RX => vec![g("Rx", params, tk_qbs(args))],
        GateType::RZ => vec![g("Rz", params, tk_qbs(args))],
        GateType::RY => vec![g("Ry", params, tk_qbs(args))],
        GateType::ZZPHASE => vec![g("ZZPhase", params, tk_qbs(args))],
        GateType::PHASEDX => vec![g("PhasedX", params, tk_qbs(args))],

        GateType::XX => vec![
            g("H", &[], vec![tk_qb(args[0])]),
            g("CX", &[], tk_qbs(args)),
            g("H", &[], vec![tk_qb(args[0])]),
        ],
        GateType::XY => vec![
            g("H", &[], vec![tk_qb(args[0])]),
            g("CY", &[], tk_qbs(args)),
            g("H", &[], vec![tk_qb(args[0])]),
        ],
        GateType::XZ => vec![g("CX", &[], vec![tk_qb(args[1]), tk_qb(args[0])])],
        GateType::YX => vec![
            g("H", &[], vec![tk_qb(args[1])]),
            g("CY", &[], vec![tk_qb(args[1]), tk_qb(args[0])]),
            g("H", &[], vec![tk_qb(args[1])]),
        ],
        GateType::YY => vec![
            g("V", &[], vec![tk_qb(args[0])]),
            g("CY", &[], tk_qbs(args)),
            g("Vdg", &[], vec![tk_qb(args[0])]),
        ],
        GateType::YZ => vec![g("CY", &[], vec![tk_qb(args[1]), tk_qb(args[0])])],
        GateType::ZX => vec![g("CX", &[], tk_qbs(args))],
        GateType::ZY => vec![g("CY", &[], tk_qbs(args))],
        GateType::ZZ => vec![g("CZ", &[], tk_qbs(args))],
        GateType::SWAP => vec![g("SWAP", &[], tk_qbs(args))],

        GateType::Measure => vec![g("Measure", &[], vec![tk_qb(args[0]), tk_b(args[1])])],
        GateType::Reset => vec![g("Reset", &[], tk_qbs(args))],
        GateType::BlackBox => {
            return Err(TKConversionError::UnsupportedPGOp(
                "BlackBox gate is not supported in TKET conversion".into(),
            ));
        }
    };
    Ok(gate_json)
}

/// Convert a `RotationData` to a TKET command, with optional conditional bits and values.
fn rotation_to_tk(
    rotation_data: &RotationData,
    cond_bits: &[Value],
    cond_values: &[bool],
) -> Value {
    let pebox_json = json!({
            "cx_config": "Tree",
            "paulis": rotation_data.get_string().iter().map(|p| {
                match p {
                    Pauli::I => "I",
                    Pauli::X => "X",
                    Pauli::Y => "Y",
                    Pauli::Z => "Z",
                }
            }).collect::<Vec<&str>>(),
            "phase": rotation_data.get_angle().to_string(),
    });
    let op_json = tk_box_json("PauliExpBox", &Uuid::new_v4().to_string(), pebox_json);
    let args_json = tk_qbs(&(0..rotation_data.get_string().len()).collect::<Vec<usize>>());
    if cond_bits.is_empty() {
        tk_cmd_json(&args_json, op_json)
    } else {
        tk_cmd_json(
            &[cond_bits, &args_json].concat(),
            tk_conditional_json(op_json, cond_bits, cond_values),
        )
    }
}

/// Convert a `TableauData` to a TKET command.
fn tableau_to_tk(tableau_data: &TableauData, n_qubits: usize) -> Value {
    let qubits = tk_qbs(&(0..n_qubits).collect::<Vec<usize>>());
    let mut x_xbits = Vec::<Vec<bool>>::with_capacity(n_qubits);
    let mut x_zbits = Vec::<Vec<bool>>::with_capacity(n_qubits);
    let mut z_xbits = Vec::<Vec<bool>>::with_capacity(n_qubits);
    let mut z_zbits = Vec::<Vec<bool>>::with_capacity(n_qubits);
    let mut phases = Vec::<Vec<bool>>::with_capacity(n_qubits);
    let process_outputs = |outputs: &Vec<(Vec<Pauli>, bool)>,
                           xbits: &mut Vec<Vec<bool>>,
                           zbits: &mut Vec<Vec<bool>>,
                           phases: &mut Vec<Vec<bool>>| {
        for (s, phase) in outputs {
            let mut s_zb = Vec::<bool>::with_capacity(n_qubits);
            let mut s_xb = Vec::<bool>::with_capacity(n_qubits);
            for p in s {
                match p {
                    Pauli::I => {
                        s_zb.push(false);
                        s_xb.push(false);
                    }
                    Pauli::X => {
                        s_zb.push(false);
                        s_xb.push(true);
                    }
                    Pauli::Y => {
                        s_zb.push(true);
                        s_xb.push(true);
                    }
                    Pauli::Z => {
                        s_zb.push(true);
                        s_xb.push(false);
                    }
                }
            }
            xbits.push(s_xb);
            zbits.push(s_zb);
            phases.push(vec![!phase]);
        }
    };
    process_outputs(
        tableau_data.get_x_outputs(),
        &mut x_xbits,
        &mut x_zbits,
        &mut phases,
    );
    process_outputs(
        tableau_data.get_z_outputs(),
        &mut z_xbits,
        &mut z_zbits,
        &mut phases,
    );
    let tab_json = json!({
        "tab": {
            "qubits": qubits,
            "tab": {
                "nqubits": n_qubits,
                "nrows": n_qubits * 2,
                "phase": phases,
                "xmat": &[x_xbits, z_xbits].concat(),
                "zmat": &[x_zbits, z_zbits].concat(),
            }
        }
    });
    tk_cmd_json(
        &qubits,
        tk_box_json("UnitaryTableauBox", &Uuid::new_v4().to_string(), tab_json),
    )
}

/// Convert an `Op` to a vector of TKET commands.
fn op_to_tk(op: &Op, n_qubits: usize) -> Result<Vec<Value>, TKConversionError> {
    match op {
        Op::Gate { data } => gate_to_tk(data),
        Op::Rotation { data } => Ok(vec![rotation_to_tk(data, &[], &[])]),
        Op::ConditionalBox { data } => data
            .get_ops()
            .iter()
            .map(|op| match op {
                Op::Rotation {
                    data: rotation_data,
                } => Ok(rotation_to_tk(
                    rotation_data,
                    &tk_bs(data.get_conditional_bits()),
                    data.get_conditional_values(),
                )),
                _ => Err(TKConversionError::UnsupportedPGOp(
                    "Only Rotation ops are supported in ConditionalBox for TK conversion".into(),
                )),
            })
            .collect(),
        Op::Tableau { data } => Ok(vec![tableau_to_tk(data, n_qubits)]),
        Op::SetBoundary => Ok(vec![]), // SetBoundary are ignored in TKET conversion
        _ => Err(TKConversionError::UnsupportedPGOp(format!(
            "Op {:?} is not supported in TK conversion",
            op
        ))),
    }
}

/// Get the set of bits used in a `PauliGraph`.
fn get_bits(pg: &PauliGraph) -> Vec<usize> {
    let mut bits = HashSet::new();
    for op in pg.get_ops() {
        match op {
            Op::Gate { data } => {
                if *data.get_gate_type() == GateType::Measure {
                    bits.insert(data.get_args()[1]);
                }
                bits.extend(data.get_conditional_bits());
            }
            Op::ConditionalBox { data } => {
                bits.extend(data.get_conditional_bits());
            }
            _ => {}
        }
    }
    let mut bits: Vec<usize> = bits.into_iter().collect();
    bits.sort();
    bits
}

/// Convert a `PauliGraph` to serialized TKET circuit format.
///
/// # Arguments
///
/// - `pg` (`&PauliGraph`) - The `PauliGraph` to convert.
///
/// # Returns
///
/// - `Result<Value, TKConversionError>` - The serialized TKET circuit format as a JSON value, or an error if the conversion fails.
///
pub fn pg_to_tk_json(pg: &PauliGraph) -> Result<Value, TKConversionError> {
    let n_qubits = pg.get_n_qubits();
    let bits = tk_bs(&get_bits(pg));
    let qubits: Vec<Value> = tk_qbs(&(0..n_qubits).collect::<Vec<usize>>());
    let commands: Vec<Value> = pg
        .get_ops()
        .iter()
        .map(|op| op_to_tk(op, n_qubits))
        .collect::<Result<Vec<Vec<Value>>, TKConversionError>>()?
        .into_iter()
        .flatten()
        .collect();
    Ok(json!({
        "qubits": qubits,
        "bits": bits,
        "commands": commands,
        "phase": "0.0",
        "created_qubits": [],
        "discarded_qubits": [],
        "implicit_permutation": qubits.iter().map(|q| vec![q.clone(), q.clone()]).collect::<Vec<Vec<Value>>>(),
    }))
}

// =============================================================================
//                   TKET to PauliGraph conversion functions
// =============================================================================

/// Convert a TKET UnitID to a PG index.
fn pg_index(unitid: &Value) -> Result<usize, TKConversionError> {
    let reg = unitid.get(0).and_then(Value::as_str).ok_or_else(|| {
        TKConversionError::InvalidTKJson("Unexpected qubit register format in TKET JSON".into())
    })?;
    if reg != "q" && reg != "c" {
        return Err(TKConversionError::UnsupportedTKJson(format!(
            "Only qubit register 'q' and bit register 'c' are supported. Found: {}",
            reg
        )));
    }
    let idx = unitid
        .get(1)
        .and_then(|v| v.get(0))
        .and_then(Value::as_u64)
        .ok_or_else(|| {
            TKConversionError::InvalidTKJson("Unexpected qubit index format in TKET JSON".into())
        })?;

    Ok(idx as usize)
}

/// Convert a TKET Pauli string to a PG Pauli.
fn pg_pauli(p: &str) -> Result<Pauli, TKConversionError> {
    match p {
        "I" => Ok(Pauli::I),
        "X" => Ok(Pauli::X),
        "Y" => Ok(Pauli::Y),
        "Z" => Ok(Pauli::Z),
        _ => Err(TKConversionError::InvalidTKJson(format!(
            "Unexpected Pauli string in TKET JSON: {}",
            p
        ))),
    }
}

/// Convert a TKET Pauli string array to a vector of PG Paulis.
fn pg_pauli_vec(pauli_list: &Value) -> Result<Vec<Pauli>, TKConversionError> {
    pauli_list
        .as_array()
        .ok_or_else(|| {
            TKConversionError::InvalidTKJson("Unexpected Pauli string format in TKET JSON".into())
        })?
        .iter()
        .map(|v| {
            pg_pauli(v.as_str().ok_or_else(|| {
                TKConversionError::InvalidTKJson(
                    "Unexpected Pauli string format in TKET JSON".into(),
                )
            })?)
        })
        .collect()
}

/// Convert pairs of Z and X bits to a vector of Paulis.
fn bits_to_pauli_vec(z_bits: &[bool], x_bits: &[bool]) -> Vec<Pauli> {
    z_bits
        .iter()
        .zip(x_bits.iter())
        .map(|(&z, &x)| match (z, x) {
            (false, false) => Pauli::I,
            (false, true) => Pauli::X,
            (true, false) => Pauli::Z,
            (true, true) => Pauli::Y,
        })
        .collect()
}

/// Convert a serialized TKET circuit to a `PauliGraph`.
///
/// # Arguments
///
/// - `tk_json` (`&Value`) - The JSON representation of the TKET circuit.
///
/// # Returns
///
/// - `Result<PauliGraph, TKConversionError>` - The resulting `PauliGraph` if the conversion is successful.
///
/// # Errors
///
/// Returns a `TKConversionError` if the TKET JSON is invalid or contains unsupported features.
pub fn pg_from_tk_json(tk_json: &Value) -> Result<PauliGraph, TKConversionError> {
    let qubits = tk_json
        .get("qubits")
        .and_then(Value::as_array)
        .ok_or_else(|| {
            TKConversionError::InvalidTKJson("Unexpected qubits format in TKET JSON".into())
        })?
        .iter()
        .map(pg_index)
        .collect::<Result<Vec<usize>, TKConversionError>>()?;
    let n_qubits = qubits.len();
    if !tk_json
        .get("created_qubits")
        .and_then(Value::as_array)
        .ok_or_else(|| {
            TKConversionError::InvalidTKJson("Unexpected created_qubits format in TKET JSON".into())
        })?
        .is_empty()
    {
        return Err(TKConversionError::UnsupportedTKJson(
            "Created qubits are not supported in TKET to PauliGraph conversion".into(),
        ));
    }
    if !tk_json
        .get("discarded_qubits")
        .and_then(Value::as_array)
        .ok_or_else(|| {
            TKConversionError::InvalidTKJson(
                "Unexpected discarded_qubits format in TKET JSON".into(),
            )
        })?
        .is_empty()
    {
        return Err(TKConversionError::UnsupportedTKJson(
            "Discarded qubits are not supported in TKET to PauliGraph conversion".into(),
        ));
    }
    if tk_json
        .get("implicit_permutation")
        .and_then(Value::as_array)
        .ok_or_else(|| {
            TKConversionError::InvalidTKJson(
                "Unexpected implicit_permutation format in TKET JSON".into(),
            )
        })?
        .iter()
        .any(|p| p[0] != p[1])
    {
        return Err(TKConversionError::UnsupportedTKJson(
            "Implicit permutation is not supported in TKET to PauliGraph conversion".into(),
        ));
    }

    let mut pg = PauliGraph::new(n_qubits);

    for cmd in tk_json
        .get("commands")
        .and_then(Value::as_array)
        .ok_or_else(|| {
            TKConversionError::InvalidTKJson("Unexpected commands format in TKET JSON".into())
        })?
    {
        let mut op = cmd.get("op").ok_or_else(|| {
            TKConversionError::InvalidTKJson("Missing op field in TKET JSON command".into())
        })?;
        let optype = op.get("type").and_then(Value::as_str).ok_or_else(|| {
            TKConversionError::InvalidTKJson("Missing or invalid type field in TKET JSON op".into())
        })?;
        let (cond_bits, cond_values) = if optype == "Conditional" {
            let conditional_json = op.get("conditional").ok_or_else(|| {
                TKConversionError::InvalidTKJson("Missing conditional field in TKET JSON op".into())
            })?;
            let width = conditional_json
                .get("width")
                .and_then(Value::as_u64)
                .ok_or_else(|| {
                    TKConversionError::InvalidTKJson(
                        "Missing or invalid width field in TKET JSON conditional".into(),
                    )
                })? as usize;
            let cond_bits = cmd
                .get("args")
                .and_then(Value::as_array)
                .ok_or_else(|| {
                    TKConversionError::InvalidTKJson(
                        "Missing or invalid args field in TKET JSON command".into(),
                    )
                })?
                .get(..width)
                .ok_or_else(|| {
                    TKConversionError::InvalidTKJson(
                        "Args field in TKET JSON command is shorter than expected width".into(),
                    )
                })?
                .iter()
                .map(pg_index)
                .collect::<Result<Vec<usize>, TKConversionError>>()?;
            // little-endian encoding of the condition values
            let cond_value_u64 = conditional_json
                .get("values")
                .and_then(Value::as_u64)
                .ok_or_else(|| {
                    TKConversionError::InvalidTKJson(
                        "Missing or invalid values field in TKET JSON conditional".into(),
                    )
                })?;
            let cond_values = (0..width)
                .map(|i| (cond_value_u64 >> i) & 1 == 1)
                .collect::<Vec<bool>>();
            op = conditional_json.get("op").ok_or_else(|| {
                TKConversionError::InvalidTKJson("Missing op field in TKET JSON conditional".into())
            })?;
            (cond_bits, cond_values)
        } else {
            (vec![], vec![])
        };
        let optype = op.get("type").and_then(Value::as_str).ok_or_else(|| {
            TKConversionError::InvalidTKJson("Missing or invalid type field in TKET JSON op".into())
        })?;
        match optype {
            "H" | "X" | "Y" | "Z" | "S" | "Sdg" | "V" | "Vdg" | "Rx" | "Ry" | "Rz" | "ZZPhase"
            | "PhasedX" | "CX" | "CY" | "CZ" | "SWAP" | "Reset" => {
                let gate_type = match optype {
                    "H" => GateType::H,
                    "X" => GateType::X,
                    "Y" => GateType::Y,
                    "Z" => GateType::Z,
                    "S" => GateType::S,
                    "Sdg" => GateType::Sdg,
                    "V" => GateType::V,
                    "Vdg" => GateType::Vdg,
                    "Rx" => GateType::RX,
                    "Ry" => GateType::RY,
                    "Rz" => GateType::RZ,
                    "ZZPhase" => GateType::ZZPHASE,
                    "PhasedX" => GateType::PHASEDX,
                    "CX" => GateType::ZX,
                    "CY" => GateType::ZY,
                    "CZ" => GateType::ZZ,
                    "SWAP" => GateType::SWAP,
                    "Reset" => GateType::Reset,
                    _ => unreachable!(),
                };
                let params = op
                    .get("params")
                    .and_then(Value::as_array)
                    .ok_or_else(|| {
                        TKConversionError::InvalidTKJson(
                            "Missing or invalid params field in TKET JSON op".into(),
                        )
                    })?
                    .iter()
                    .map(|p| {
                        p.as_str()
                            .and_then(|s| s.parse::<f64>().ok())
                            .ok_or_else(|| {
                                TKConversionError::InvalidTKJson(
                                    "Invalid param format in TKET JSON".into(),
                                )
                            })
                    })
                    .collect::<Result<Vec<f64>, TKConversionError>>()?;
                let args = cmd.get("args").and_then(Value::as_array).ok_or_else(|| {
                    TKConversionError::InvalidTKJson(
                        "Missing or invalid args field in TKET JSON op".into(),
                    )
                })?[cond_bits.len()..]
                    .iter()
                    .map(pg_index)
                    .collect::<Result<Vec<usize>, TKConversionError>>()?;
                let gate_data = GateData::new(gate_type, args)
                    .with_params(params)
                    .with_conditional(cond_bits, cond_values);
                pg.add_op(Op::Gate { data: gate_data });
            }
            "Measure" => {
                let args = cmd.get("args").and_then(Value::as_array).ok_or_else(|| {
                    TKConversionError::InvalidTKJson(
                        "Missing or invalid args field in TKET JSON op".into(),
                    )
                })?[cond_bits.len()..]
                    .iter()
                    .map(pg_index)
                    .collect::<Result<Vec<usize>, TKConversionError>>()?;
                if args.len() != 2 {
                    return Err(TKConversionError::InvalidTKJson(
                        "Measure gate must have 2 arguments".into(),
                    ));
                }
                let gate_data =
                    GateData::new(GateType::Measure, args).with_conditional(cond_bits, cond_values);
                pg.add_op(Op::Gate { data: gate_data });
            }
            "PauliExpBox" => {
                let paulis_json = op.get("box").and_then(|b| b.get("paulis")).ok_or_else(|| {
                    TKConversionError::InvalidTKJson("Missing paulis field in PauliExpBox".into())
                })?;
                let paulis = pg_pauli_vec(paulis_json)?;
                let phase_json = op
                    .get("box")
                    .and_then(|b| b.get("phase"))
                    .and_then(Value::as_str)
                    .and_then(|s| s.parse::<f64>().ok())
                    .ok_or_else(|| {
                        TKConversionError::InvalidTKJson(
                            "Missing or invalid phase field in PauliExpBox".into(),
                        )
                    })?;
                let rotation_data = RotationData::new(paulis, phase_json);
                pg.add_conditional_op(
                    Op::Rotation {
                        data: rotation_data,
                    },
                    cond_bits,
                    cond_values,
                );
            }
            "UnitaryTableauBox" => {
                if !cond_bits.is_empty() {
                    return Err(TKConversionError::UnsupportedTKJson("Conditional UnitaryTableauBox is not supported in TKET to PauliGraph conversion".into()));
                }
                let tab = &op
                    .get("box")
                    .and_then(|b| b.get("tab"))
                    .and_then(|b| b.get("tab"))
                    .ok_or_else(|| {
                        TKConversionError::InvalidTKJson(
                            "Missing tab field in UnitaryTableauBox".into(),
                        )
                    })?;
                let tab_width = tab.get("nqubits").and_then(Value::as_u64).ok_or_else(|| {
                    TKConversionError::InvalidTKJson(
                        "Missing or invalid nqubits field in UnitaryTableauBox".into(),
                    )
                })? as usize;
                if tab_width != n_qubits {
                    return Err(TKConversionError::UnsupportedTKJson(format!(
                        "Currently only support tableau with the same number of qubits as the circuit. Found: {}. Expected: {}",
                        tab_width, n_qubits
                    )));
                }
                // x bits. x outputs followed by z outputs.
                let xmat = tab
                    .get("xmat")
                    .and_then(Value::as_array)
                    .ok_or_else(|| {
                        TKConversionError::InvalidTKJson(
                            "Missing or invalid xmat field in UnitaryTableauBox".into(),
                        )
                    })?
                    .iter()
                    .map(|row| {
                        row.as_array()
                            .ok_or_else(|| {
                                TKConversionError::InvalidTKJson(
                                    "Unexpected xmat row format in TKET JSON".into(),
                                )
                            })?
                            .iter()
                            .map(|b| {
                                b.as_bool().ok_or_else(|| {
                                    TKConversionError::InvalidTKJson(
                                        "Unexpected xmat bit format in TKET JSON".into(),
                                    )
                                })
                            })
                            .collect::<Result<Vec<bool>, TKConversionError>>()
                    })
                    .collect::<Result<Vec<Vec<bool>>, TKConversionError>>()?;
                // z bits. x outputs followed by z outputs.
                let zmat = tab
                    .get("zmat")
                    .and_then(Value::as_array)
                    .ok_or_else(|| {
                        TKConversionError::InvalidTKJson(
                            "Missing or invalid zmat field in UnitaryTableauBox".into(),
                        )
                    })?
                    .iter()
                    .map(|row| {
                        row.as_array()
                            .ok_or_else(|| {
                                TKConversionError::InvalidTKJson(
                                    "Unexpected zmat row format in TKET JSON".into(),
                                )
                            })?
                            .iter()
                            .map(|b| {
                                b.as_bool().ok_or_else(|| {
                                    TKConversionError::InvalidTKJson(
                                        "Unexpected zmat bit format in TKET JSON".into(),
                                    )
                                })
                            })
                            .collect::<Result<Vec<bool>, TKConversionError>>()
                    })
                    .collect::<Result<Vec<Vec<bool>>, TKConversionError>>()?;
                let phases = tab
                    .get("phase")
                    .and_then(Value::as_array)
                    .ok_or_else(|| {
                        TKConversionError::InvalidTKJson(
                            "Missing or invalid phase field in UnitaryTableauBox".into(),
                        )
                    })?
                    .iter()
                    .map(|row| {
                        row.as_array().ok_or_else(|| {
                            TKConversionError::InvalidTKJson(
                                "Unexpected phase row format in TKET JSON".into(),
                            )
                        })?[0]
                            .as_bool()
                            .ok_or_else(|| {
                                TKConversionError::InvalidTKJson(
                                    "Unexpected phase bit format in TKET JSON".into(),
                                )
                            })
                    })
                    .collect::<Result<Vec<bool>, TKConversionError>>()?;
                let mut x_outputs = Vec::with_capacity(n_qubits);
                let mut z_outputs = Vec::with_capacity(n_qubits);
                for i in 0..n_qubits {
                    let x_output_xbits = &xmat[i];
                    let z_output_xbits = &xmat[i + n_qubits];
                    let x_output_zbits = &zmat[i];
                    let z_output_zbits = &zmat[i + n_qubits];
                    // tket produces true for -1 phase
                    let x_output_phase = !phases[i];
                    let z_output_phase = !phases[i + n_qubits];
                    let x_output = (
                        bits_to_pauli_vec(x_output_zbits, x_output_xbits),
                        x_output_phase,
                    );
                    let z_output = (
                        bits_to_pauli_vec(z_output_zbits, z_output_xbits),
                        z_output_phase,
                    );
                    x_outputs.push(x_output);
                    z_outputs.push(z_output);
                }
                let tableau_data = TableauData::new(z_outputs, x_outputs);
                pg.add_op(Op::Tableau { data: tableau_data });
            }
            _ => {
                return Err(TKConversionError::UnsupportedTKJson(format!(
                    "Unsupported op type in TKET JSON: {}",
                    optype
                )));
            }
        }
    }
    Ok(pg)
}

#[cfg(test)]
mod tests {
    use super::*;
    use pg_core::{BlackBoxData, ConditionalBoxData};
    fn remove_box_id(value: &mut Value) {
        value["commands"]
            .as_array_mut()
            .unwrap()
            .iter_mut()
            .for_each(|cmd| {
                if let Some(op) = cmd.get_mut("op") {
                    if let Some(box_json) = op.get_mut("box") {
                        box_json.as_object_mut().unwrap().remove("id");
                    }
                    if let Some(condition_json) = op.get_mut("conditional")
                        && let Some(box_json) = condition_json.get_mut("op")
                        && let Some(box_json) = box_json.get_mut("box")
                    {
                        box_json.as_object_mut().unwrap().remove("id");
                    }
                }
            });
    }

    #[test]
    fn test_simple_pg_to_tk_json() {
        let mut pg = PauliGraph::new(2);
        pg.add_op(Op::Gate {
            data: GateData::new(GateType::H, vec![0]),
        });
        pg.add_op(Op::Gate {
            data: GateData::new(GateType::Measure, vec![0, 1]),
        });
        pg.add_op(Op::Gate {
            data: GateData::new(GateType::Reset, vec![1]),
        });
        pg.add_op(Op::Gate {
            data: GateData::new(GateType::PHASEDX, vec![1]).with_params(vec![0.4, 0.4]),
        });
        pg.add_op(Op::Gate {
            data: GateData::new(GateType::X, vec![0])
                .with_conditional(vec![1, 0], vec![true, false]),
        });
        let tk_json = pg_to_tk_json(&pg).unwrap();
        let expected_json = json!({
            "qubits": [["q", [0]], ["q", [1]]],
            "bits": [["c", [0]], ["c", [1]]],
            "commands": [
                {
                    "op": {
                        "type": "H",
                        "params": []
                    },
                    "args": [["q", [0]]]
                },
                {
                    "op": {
                        "type": "Measure",
                        "params": []
                    },
                    "args": [["q", [0]], ["c", [1]]]
                },
                {
                    "op": {
                        "type": "Reset",
                        "params": []
                    },
                    "args": [["q", [1]]]
                },
                {
                    "op": {
                        "type": "PhasedX",
                        "params": ["0.4", "0.4"]
                    },
                    "args": [["q", [1]]]
                },
                {
                    "op": {
                        "conditional": {
                            "op": {
                                "type": "X",
                                "params": []
                            },
                            "width": 2,
                            "values": 1
                        },
                        "type": "Conditional"
                    },
                    "args": [
                        ["c", [1]],
                        ["c", [0]],
                        ["q", [0]]
                    ]
                }
            ],
            "phase": "0.0",
            "created_qubits": [],
            "discarded_qubits": [],
            "implicit_permutation": [
                [["q", [0]], ["q", [0]]],
                [["q", [1]], ["q", [1]]]
            ]
        });
        assert_eq!(tk_json, expected_json);
        // test round trip conversion
        let pg2 = pg_from_tk_json(&tk_json).unwrap();
        assert_eq!(pg.get_n_qubits(), pg2.get_n_qubits());
        assert_eq!(pg.get_ops(), pg2.get_ops());
    }

    #[test]
    fn test_tqe_to_tk_json() {
        let mut pg = PauliGraph::new(2);
        pg.add_op(Op::Gate {
            data: GateData::new(GateType::XX, vec![0, 1]),
        });
        let tk_json = pg_to_tk_json(&pg);
        let expected_json = json!({
            "qubits": [["q", [0]], ["q", [1]]],
            "bits": [],
            "commands": [
                {
                    "op": {
                        "type": "H",
                        "params": []
                    },
                    "args": [["q", [0]]]
                },
                {
                    "op": {
                        "type": "CX",
                        "params": []
                    },
                    "args": [["q", [0]], ["q", [1]]]
                },
                {
                    "op": {
                        "type": "H",
                        "params": []
                    },
                    "args": [["q", [0]]]
                }
            ],
            "phase": "0.0",
            "created_qubits": [],
            "discarded_qubits": [],
            "implicit_permutation": [
                [["q", [0]], ["q", [0]]],
                [["q", [1]], ["q", [1]]]
            ]
        });
        assert_eq!(tk_json.unwrap(), expected_json);
    }

    #[test]
    fn test_rotation_to_tk() {
        let mut pg = PauliGraph::new(3);
        let rotation_data = RotationData::new(vec![Pauli::X, Pauli::Y, Pauli::Z], 0.5);
        pg.add_op(Op::Rotation {
            data: rotation_data,
        });
        let mut tk_json = pg_to_tk_json(&pg).unwrap();
        let expected_json = json!({
            "qubits": [["q", [0]], ["q", [1]], ["q", [2]]],
            "bits": [],
            "commands": [
                {
                    "op": {
                        "box": {
                            "cx_config": "Tree",
                            "paulis": ["X", "Y", "Z"],
                            "phase": "0.5",
                            "type": "PauliExpBox"
                        },
                        "type": "PauliExpBox"
                    },
                    "args": [["q", [0]], ["q", [1]], ["q", [2]]]
                }
            ],
            "phase": "0.0",
            "created_qubits": [],
            "discarded_qubits": [],
            "implicit_permutation": [
                [["q", [0]], ["q", [0]]],
                [["q", [1]], ["q", [1]]],
                [["q", [2]], ["q", [2]]]
            ]
        });
        remove_box_id(&mut tk_json);
        assert_eq!(tk_json, expected_json);
        // test round trip conversion
        let pg2 = pg_from_tk_json(&tk_json).unwrap();
        assert_eq!(pg.get_n_qubits(), pg2.get_n_qubits());
        assert_eq!(pg.get_ops(), pg2.get_ops());
    }

    #[test]
    fn test_conditional_box_pg_to_tk_json() {
        let mut pg = PauliGraph::new(1);
        let rotation_data = RotationData::new(vec![Pauli::X], 0.5);
        let conditional_box_data = ConditionalBoxData::new(
            vec![Op::Rotation {
                data: rotation_data,
            }],
            vec![0, 1, 2],
            vec![true, false, true],
        );
        pg.add_op(Op::ConditionalBox {
            data: conditional_box_data,
        });
        let mut tk_json = pg_to_tk_json(&pg).unwrap();
        let expected_json = json!({
            "qubits": [["q", [0]]],
            "bits": [["c", [0]], ["c", [1]], ["c", [2]]],
            "commands": [
                {
                    "op": {
                        "conditional": {
                            "op": {
                                "type": "PauliExpBox",
                                "box": {
                                    "cx_config": "Tree",
                                    "paulis": ["X"],
                                    "phase": "0.5",
                                    "type": "PauliExpBox"
                                }
                            },
                            "width": 3,
                            "values": 5
                        },
                        "type": "Conditional"
                    },
                    "args": [
                        ["c", [0]],
                        ["c", [1]],
                        ["c", [2]],
                        ["q", [0]]
                    ]
                }
            ],
            "phase": "0.0",
            "created_qubits": [],
            "discarded_qubits": [],
            "implicit_permutation": [
                [["q", [0]], ["q", [0]]]
            ]
        });

        remove_box_id(&mut tk_json);
        assert_eq!(tk_json, expected_json);
        // test round trip conversion
        let pg2 = pg_from_tk_json(&tk_json).unwrap();
        assert_eq!(pg.get_n_qubits(), pg2.get_n_qubits());
        assert_eq!(pg.get_ops(), pg2.get_ops());
    }

    #[test]
    fn test_tableau_to_tk() {
        let mut pg = PauliGraph::new(2);
        // cx(0,1);z(0)
        let tableau_data = TableauData::new(
            vec![
                (vec![Pauli::Z, Pauli::I], true),
                (vec![Pauli::Z, Pauli::Z], true),
            ],
            vec![
                (vec![Pauli::X, Pauli::X], false),
                (vec![Pauli::I, Pauli::X], true),
            ],
        );
        pg.add_op(Op::Tableau { data: tableau_data });
        let mut tk_json = pg_to_tk_json(&pg).unwrap();
        let expected_json = json!({
            "qubits": [["q", [0]], ["q", [1]]],
            "bits": [],
            "commands": [
                {
                    "op": {
                        "box": {
                            "tab": {
                                "qubits": [["q", [0]], ["q", [1]]],
                                "tab": {
                                    "nqubits": 2,
                                    "nrows": 4,
                                    "phase": [[true], [false], [false], [false]],
                                    "xmat": [[true, true], [false, true], [false, false], [false, false]],
                                    "zmat": [[false, false], [false, false], [true, false], [true, true]]
                                }
                            },
                            "type": "UnitaryTableauBox"
                        },
                        "type": "UnitaryTableauBox"
                    },
                    "args": [["q", [0]], ["q", [1]]]
                }
            ],
            "phase": "0.0",
            "created_qubits": [],
            "discarded_qubits": [],
            "implicit_permutation": [
                [["q", [0]], ["q", [0]]],
                [["q", [1]], ["q", [1]]]
            ]
        });
        remove_box_id(&mut tk_json);
        assert_eq!(tk_json, expected_json);
        // test round trip conversion
        let pg2 = pg_from_tk_json(&tk_json).unwrap();
        assert_eq!(pg.get_n_qubits(), pg2.get_n_qubits());
        assert_eq!(pg.get_ops(), pg2.get_ops());
    }

    #[test]
    fn test_convert_blackbox_fails() {
        let mut pg = PauliGraph::new(2);
        pg.add_op(Op::BlackBox {
            data: BlackBoxData::new(vec![0, 1], "Blackbox".into()),
        });
        let tk_json = pg_to_tk_json(&pg);
        assert!(tk_json.is_err());
    }

    #[test]
    fn test_convert_malformed_tk_json_fails() {
        let tk_json = json!({
            "qubits": [["q", [0]], ["q", [1]]],
            "bits": [],
            "commands": [
                {
                    "op": {
                        "type": "H",
                        "params": []
                    },
                    "args": [["q", [0]]]
                },
                {
                    "op": {
                        "type": "Measure",
                        "params": []
                    },
                    // Missing args field
                }
            ],
            "phase": "0.0",
            "created_qubits": [],
            "discarded_qubits": [],
            "implicit_permutation": [
                [["q", [0]], ["q", [0]]],
                [["q", [1]], ["q", [1]]]
            ]
        });
        let pg_result = pg_from_tk_json(&tk_json);
        assert!(pg_result.is_err());
    }

    #[test]
    fn test_set_boundary_ignored() {
        let mut pg = PauliGraph::new(1);
        pg.add_op(Op::SetBoundary);
        let tk_json = pg_to_tk_json(&pg).unwrap();
        let expected_json = json!({
            "qubits": [["q", [0]]],
            "bits": [],
            "commands": [],
            "phase": "0.0",
            "created_qubits": [],
            "discarded_qubits": [],
            "implicit_permutation": [
                [["q", [0]], ["q", [0]]]
            ]
        });
        assert_eq!(tk_json, expected_json);
    }
}
