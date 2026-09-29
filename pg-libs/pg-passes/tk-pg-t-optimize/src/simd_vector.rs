/// A wrapper type to enforce 32-byte alignment for SIMD loads
#[repr(align(32))]
struct Lanes([i32; 8]);

#[derive(Debug, Clone, Copy)]
pub struct Block {
    #[cfg(target_feature = "avx2")]
    inner: std::arch::x86_64::__m256i,

    #[cfg(target_feature = "neon")]
    inner: [std::arch::aarch64::int32x4_t; 2],

    #[cfg(not(any(target_feature = "avx2", target_feature = "neon")))]
    inner: [i32; 8],
}

impl Block {
    #[cfg(target_feature = "neon")]
    fn constant(a: i32) -> Self {
        Block {
            inner: unsafe { [std::arch::aarch64::vdupq_n_s32(a); 2] },
        }
    }

    #[cfg(not(any(target_feature = "avx2", target_feature = "neon")))]
    fn constant(a: i32) -> Self {
        Block { inner: [a; 8] }
    }

    #[cfg(target_feature = "avx2")]
    fn load(arr: &Lanes) -> Self {
        Block {
            inner: unsafe { std::arch::x86_64::_mm256_load_si256(arr.0.as_ptr() as *const _) },
        }
    }

    #[cfg(target_feature = "neon")]
    fn load(arr: &Lanes) -> Self {
        Block {
            inner: unsafe {
                [
                    std::arch::aarch64::vld1q_s32(&arr.0[0]),
                    std::arch::aarch64::vld1q_s32(&arr.0[4]),
                ]
            },
        }
    }

    #[cfg(not(any(target_feature = "avx2", target_feature = "neon")))]
    fn load(arr: &Lanes) -> Self {
        Block {
            inner: arr.0.clone(),
        }
    }

    #[cfg(target_feature = "avx2")]
    fn zero() -> Self {
        Block {
            inner: unsafe { std::arch::x86_64::_mm256_setzero_si256() },
        }
    }

    #[cfg(target_feature = "neon")]
    fn zero() -> Self {
        Block::constant(0)
    }

    #[cfg(not(any(target_feature = "avx2", target_feature = "neon")))]
    fn zero() -> Self {
        Self::constant(0)
    }

    #[cfg(target_feature = "avx2")]
    fn extract(&self) -> [i32; 8] {
        let mut arr = Lanes([0; 8]);
        unsafe {
            std::arch::x86_64::_mm256_store_si256(arr.0.as_mut_ptr() as *mut _, self.inner);
        }
        arr.0
    }

    #[cfg(target_feature = "neon")]
    fn extract(&self) -> [i32; 8] {
        let mut arr = Lanes([0; 8]);
        unsafe {
            std::arch::aarch64::vst1q_s32(arr.0.as_mut_ptr(), self.inner[0]);
            std::arch::aarch64::vst1q_s32(arr.0.as_mut_ptr().add(4), self.inner[1]);
        }
        arr.0
    }

    #[cfg(not(any(target_feature = "avx2", target_feature = "neon")))]
    fn extract(&self) -> [i32; 8] {
        self.inner
    }
}

impl std::ops::BitXorAssign for Block {
    #[cfg(target_feature = "avx2")]
    fn bitxor_assign(&mut self, rhs: Self) {
        self.inner = unsafe { std::arch::x86_64::_mm256_xor_si256(self.inner, rhs.inner) };
    }

    #[cfg(target_feature = "neon")]
    fn bitxor_assign(&mut self, rhs: Self) {
        self.inner[0] = unsafe { std::arch::aarch64::veorq_s32(self.inner[0], rhs.inner[0]) };
        self.inner[1] = unsafe { std::arch::aarch64::veorq_s32(self.inner[1], rhs.inner[1]) };
    }

    #[cfg(not(any(target_feature = "avx2", target_feature = "neon")))]
    fn bitxor_assign(&mut self, rhs: Self) {
        for i in 0..8 {
            self.inner[i] ^= rhs.inner[i];
        }
    }
}

impl std::ops::BitAndAssign for Block {
    fn bitand_assign(&mut self, rhs: Self) {
        #[cfg(target_feature = "avx2")]
        {
            self.inner = unsafe { std::arch::x86_64::_mm256_and_si256(self.inner, rhs.inner) };
        }
        #[cfg(target_feature = "neon")]
        {
            self.inner[0] = unsafe { std::arch::aarch64::vandq_s32(self.inner[0], rhs.inner[0]) };
            self.inner[1] = unsafe { std::arch::aarch64::vandq_s32(self.inner[1], rhs.inner[1]) };
        }
        #[cfg(not(any(target_feature = "avx2", target_feature = "neon")))]
        for i in 0..8 {
            self.inner[i] &= rhs.inner[i];
        }
    }
}

#[derive(Debug, Clone)]
pub struct SIMDVector {
    pub blocks: Vec<Block>,
}

impl SIMDVector {
    const LANES: usize = 8;
    const LANE_SIZE: usize = 32;
    const BLOCK_SIZE: usize = 256;

    pub fn new(nb_bits: usize) -> Self {
        SIMDVector {
            blocks: SIMDVector::init_blocks(nb_bits),
        }
    }

    fn init_blocks(nb_bits: usize) -> Vec<Block> {
        let capacity = nb_bits / SIMDVector::BLOCK_SIZE + 1;
        let mut vec: Vec<Block> = Vec::with_capacity(capacity);
        for _ in 0..vec.capacity() {
            vec.push(Block::zero());
        }
        vec
    }

    pub fn flip_bit(&mut self, mut bit: usize) {
        let block_index = bit / SIMDVector::BLOCK_SIZE;
        bit = bit % SIMDVector::BLOCK_SIZE;
        let lane_index = bit / SIMDVector::LANE_SIZE;
        bit = bit % SIMDVector::LANE_SIZE;
        let mut arr = Lanes([0; SIMDVector::LANES]);
        arr.0[lane_index] ^= 1 << bit;
        self.blocks[block_index] ^= Block::load(&arr);
    }

    pub fn get(&self, mut bit: usize) -> bool {
        let block_index = bit / SIMDVector::BLOCK_SIZE;
        bit = bit % SIMDVector::BLOCK_SIZE;
        let lane_index = bit / SIMDVector::LANE_SIZE;
        bit = bit % SIMDVector::LANE_SIZE;
        self.extract_block(block_index)[lane_index] & (1 << bit) != 0
    }

    pub fn first_one(&self) -> Option<usize> {
        for (block_index, block) in self.blocks.iter().enumerate() {
            for (lane, word) in block.extract().iter().enumerate() {
                if *word != 0 {
                    return Some(
                        block_index * Self::BLOCK_SIZE
                            + lane * Self::LANE_SIZE
                            + word.trailing_zeros() as usize,
                    );
                }
            }
        }
        None
    }

    pub fn packed_words(&self) -> Vec<i32> {
        self.blocks.iter().flat_map(|b| b.extract()).collect()
    }

    pub fn dot_parity(&self, other: &Self) -> bool {
        let mut parity = 0;
        for (a, b) in self.blocks.iter().zip(&other.blocks) {
            for (x, y) in a.extract().iter().zip(b.extract()) {
                parity ^= (x & y).count_ones() & 1;
            }
        }
        parity != 0
    }

    pub fn and(&mut self, other: &Self) {
        for (a, b) in self.blocks.iter_mut().zip(&other.blocks) {
            *a &= *b;
        }
    }

    pub fn xor(&mut self, bv: &SIMDVector) {
        for i in 0..self.blocks.len() {
            self.blocks[i] ^= bv.blocks[i];
        }
    }

    pub fn popcount(&self) -> i32 {
        let mut sum: i32 = 0;
        for block_index in 0..self.blocks.len() {
            let arr = self.extract_block(block_index);
            for j in 0..8 {
                sum += arr[j].count_ones() as i32;
            }
        }
        sum
    }

    fn extract_block(&self, block: usize) -> [i32; 8] {
        self.blocks[block].extract()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new() {
        let vec = SIMDVector::new(100);
        assert_eq!(vec.blocks.len(), 1);
    }

    #[test]
    fn test_flip_bit() {
        let mut vec = SIMDVector::new(10);
        vec.flip_bit(5);
        assert!(vec.get(5));
    }

    #[test]
    fn test_first_one() {
        let mut vec = SIMDVector::new(100);
        vec.flip_bit(42);
        assert_eq!(vec.first_one(), Some(42));
    }

    #[test]
    fn test_xor() {
        let mut vec1 = SIMDVector::new(10);
        let mut vec2 = SIMDVector::new(10);

        vec1.flip_bit(5);
        vec2.flip_bit(7);
        vec1.xor(&vec2);

        assert!(vec1.get(5));
        assert!(vec1.get(7));
        assert_eq!(vec1.popcount(), 2);
    }

    #[test]
    fn test_popcount() {
        let mut vec = SIMDVector::new(10);
        vec.flip_bit(2);
        vec.flip_bit(6);
        vec.flip_bit(7);
        assert_eq!(vec.popcount(), 3);
    }
    #[test]
    fn packed_operations_match_scalar_bits() {
        let width = 777;
        let mut a = SIMDVector::new(width);
        let mut b = SIMDVector::new(width);
        for q in 0..width {
            if q % 3 == 0 {
                a.flip_bit(q);
            }
            if q % 7 == 0 {
                b.flip_bit(q);
            }
        }
        let mut intersection = a.clone();
        intersection.and(&b);
        for q in 0..width {
            assert_eq!(intersection.get(q), q % 21 == 0);
            let words = a.packed_words();
            assert_eq!(words[q / 32] as u32 & (1 << (q % 32)) != 0, a.get(q));
        }
        assert_eq!(
            a.dot_parity(&b),
            (0..width).filter(|q| q % 21 == 0).count() % 2 != 0
        );
        let mut single = SIMDVector::new(width);
        assert_eq!(single.first_one(), None);
        for q in [0, 31, 32, 63, 64, 255, 256, 512, 776] {
            single.flip_bit(q);
            assert_eq!(single.first_one(), Some(q));
            single.flip_bit(q);
        }
        assert_eq!(single.first_one(), None);
    }
}
