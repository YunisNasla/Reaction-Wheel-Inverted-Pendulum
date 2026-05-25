import { Routes, Route } from 'react-router-dom'

export default function App() {
  return (
    <Routes>
      <Route path="/" element={
        <section className="flex flex-col items-center justify-center text-center px-8 py-40">
          <h1 className="font-display text-8xl leading-none tracking-tighter text-ink max-w-4xl">
            Reaction Wheel Inverted Pendulum
          </h1>
          <p className="font-display text-lg text-ink mt-2 font-medium max-w-pill">
            A self-balancing inverted pendulum that swings itself and balances upright from rest.
            Rejecting any external disturbances using only a spinning reaction wheel. <br />
            No contact with the ground, no external support, just control theory.
          </p>
          <div className="flex items-center gap-4 mt-4">
            <button className="font-display bg-near-black text-canvas text-[17px] font-bold px-4 py-2 rounded-full">
              Explore how it works
            </button>
            <button className="font-display text-ink font-medium text-lg border-b border-ink pb-0.5">
              Live Control
            </button>
          </div>
        </section>
      } />
    </Routes>
  )
}
