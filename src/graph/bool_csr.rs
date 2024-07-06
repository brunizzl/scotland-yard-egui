use itertools::Itertools;
use serde::{Deserialize, Serialize};

/// stores a matrix of bool entries in CSR format.
/// because CSR stores only nonzero entries and there is only one nonzero entry in bool,
/// no actual storage of the elements is required, only where they are.
#[derive(Clone, Serialize, Deserialize)]
pub struct BoolCSR {
    row_offsets: Vec<usize>,
    col_indices: Vec<usize>,
}

impl BoolCSR {
    pub fn nr_rows(&self) -> usize {
        self.row_offsets.len() - 1
    }

    pub fn nr_entries(&self) -> usize {
        self.col_indices.len()
    }

    pub fn new() -> Self {
        Self {
            row_offsets: vec![0],
            col_indices: Vec::new(),
        }
    }

    pub fn start_new_row(&mut self) {
        self.row_offsets.push(self.nr_entries());
    }

    pub fn add_entry_in_last_row(&mut self, col: usize) {
        assert!(self.nr_rows() > 0);
        *self.row_offsets.last_mut().unwrap() += 1;
        self.col_indices.push(col);
    }

    pub fn add_row(&mut self, cols: impl Iterator<Item = usize>) {
        self.start_new_row();
        for col in cols {
            self.add_entry_in_last_row(col);
        }
    }

    pub fn iter_rows(&self) -> impl Iterator<Item = &'_ [usize]> + Clone {
        self.row_offsets
            .iter()
            .tuple_windows()
            .map(|(&start, &end)| &self.col_indices[start..end])
    }

    pub fn last_row(&self) -> &[usize] {
        assert!(self.nr_rows() > 0);
        let len = self.row_offsets.len();
        let start = self.row_offsets[len - 2];
        let end = self.row_offsets[len - 1];
        &self.col_indices[start..end]
    }

    pub fn row(&self, row: usize) -> &[usize] {
        assert!(self.nr_rows() > row);
        let start = self.row_offsets[row];
        let end = self.row_offsets[row + 1];
        &self.col_indices[start..end]
    }
}

impl std::ops::Index<usize> for BoolCSR {
    type Output = [usize];

    fn index(&self, index: usize) -> &Self::Output {
        self.row(index)
    }
}

#[cfg(test)]
mod test {

    use super::*;
    use itertools::izip;

    #[test]
    fn construct_csr() {
        let mut mat = BoolCSR::new();
        let rows = [[1, 2], [2, 3], [0, 4], [0, 1], [8, 9]];
        for row in &rows {
            mat.add_row(row.iter().copied());
        }
        assert_eq!(mat.nr_rows(), rows.len());
        assert_eq!(mat.nr_entries(), rows.iter().map(|r| r.len()).sum());
        for (row, row_m) in izip!(&rows, mat.iter_rows()) {
            for (&col, &col_m) in izip!(row, row_m) {
                assert_eq!(col, col_m);
            }
        }

        for (c, c_m) in izip!(rows.last().unwrap(), mat.last_row()) {
            assert_eq!(c, c_m);
        }
    }
}
