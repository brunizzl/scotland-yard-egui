
use itertools::Itertools;
use serde::{Deserialize, Serialize};

/// stores a matrix of bool entries in CSR format.
/// because CSR stores only nonzero entries and there is only one nonzero entry in bool,
/// no actual storage of the elements is required, only where they are.
#[derive(Clone)]
#[derive(Serialize, Deserialize)]
pub struct BoolCSR {
    nr_cols: usize,
    row_offsets: Vec<usize>,
    col_indices: Vec<usize>,
}

impl BoolCSR {
    pub fn nr_cols(&self) -> usize {
        self.nr_cols
    }

    pub fn nr_rows(&self) -> usize {
        self.row_offsets.len() - 1
    }

    pub fn nr_entries(&self) -> usize {
        self.col_indices.len()
    }

    pub fn new(nr_cols: usize) -> Self {
        Self { nr_cols, row_offsets: vec![0], col_indices: Vec::new() }
    }

    pub fn add_row(&mut self) {
        self.row_offsets.push(self.nr_entries());
    }

    pub fn add_entry_in_last_row(&mut self, col: usize) {
        assert!(self.nr_rows() > 0);
        assert!(col < self.nr_cols);
        *self.row_offsets.last_mut().unwrap() += 1;
        self.col_indices.push(col);
    }

    pub fn iter_rows<'a>(&'a self) -> impl Iterator<Item = &'a [usize]> + Clone {
        self.row_offsets.iter().tuple_windows().map(
            |(&start, &end)| &self.col_indices[start..end]
        )
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


#[cfg(test)]
mod test {

    use super::*;
    use itertools::izip;

    #[test]
    fn construct_csr() {
        let mut mat = BoolCSR::new(10);
        let rows = [[1, 2], [2, 3], [0, 4], [0, 1], [8, 9]];
        for row in &rows {
            mat.add_row();
            for &col in row {
                mat.add_entry_in_last_row(col);
            }
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


