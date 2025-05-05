use itertools::Itertools;

/// creates run-length encoding of input
pub fn encode<T: Eq + Clone>(input: &[T]) -> Vec<(usize, T)> {
    let mut res = Vec::new();
    if input.is_empty() {
        return res;
    }
    let mut curr = &input[0];
    let mut run = 1;
    for elem in &input[1..] {
        if elem == curr {
            run += 1;
        } else {
            res.push((run, curr.clone()));
            curr = elem;
            run = 1;
        }
    }
    res.push((run, curr.clone()));
    res
}

pub fn decode<T: Clone>(msg: &[(usize, T)]) -> Vec<T> {
    msg.iter()
        .flat_map(|(run, val)| std::iter::repeat_n(val.clone(), *run))
        .collect_vec()
}

#[cfg(test)]
mod test {

    use super::*;

    #[test]
    fn roundtrip_1() {
        let original = vec![0, 1, 1, 1, 0, 1092386713, 2, 2, 0, 0, 0, 0, 0];
        let encoded = encode(&original);
        assert_eq!(
            encoded,
            vec![(1, 0), (3, 1), (1, 0), (1, 1092386713), (2, 2), (5, 0)]
        );
        let decoded = decode(&encoded);
        assert_eq!(original, decoded);
    }

    #[test]
    fn roundtrip_2() {
        let original = vec![
            2, 4, 4, 4, 6, 6, 6, 6, 6, 8, 8, 8, 8, 8, 8, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        ];
        let encoded = encode(&original);
        assert_eq!(encoded, vec![(1, 2), (3, 4), (5, 6), (7, 8), (9, 0)]);
        let decoded = decode(&encoded);
        assert_eq!(original, decoded);
    }
}
