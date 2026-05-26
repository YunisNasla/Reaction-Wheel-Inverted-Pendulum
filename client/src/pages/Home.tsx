import { Link } from 'react-router-dom'

function scrollToCentered(id: string, duration = 1600) {
  const el = document.getElementById(id)
  if (!el) return
  const rect = el.getBoundingClientRect()
  const startY = window.scrollY
  const viewH = window.innerHeight
  const elH = rect.height
  const targetY = startY + rect.top - Math.max((viewH - elH) / 2, 0)
  const diff = targetY - startY
  const start = performance.now()

  function step(ts: number) {
    const t = Math.min((ts - start) / duration, 1)
    const ease = t < 0.5 ? 4 * t ** 3 : 1 - (-2 * t + 2) ** 3 / 2
    window.scrollTo(0, startY + diff * ease)
    if (t < 1) requestAnimationFrame(step)
  }

  requestAnimationFrame(step)
}

export default function Home() {
  return (
    <main className="bg-canvas flex flex-col items-center px-8">
      <section className="min-h-screen w-full max-w-4xl flex flex-col items-center justify-center text-center gap-6 py-32">

        <h1 className="font-display text-[96px] leading-none tracking-[-1.92px] text-ink">
          Reaction Wheel Inverted Pendulum
        </h1>

        <p className="font-display text-[22px] leading-[1.4] text-ink max-w-1.5xl">
          A self-balancing inverted pendulum that swings itself and balances upright from rest.
          Rejecting any external disturbances using only a spinning reaction wheel. <br />
          No contact with the ground, no external support, just control theory.
        </p>

        <div className="flex items-center gap-5 mt-2">
          <button
            onClick={() => scrollToCentered('how-it-works')}
            className="font-display bg-near-black text-canvas text-[17px] font-medium px-5 py-3.5 rounded-[32px] hover:opacity-80 transition-opacity"
          >
            Explore how it works
          </button>
          <Link
            to="/control"
            className="font-display text-ink text-[19px] border-b border-ink pb-0.5 hover:text-muted-slate hover:border-muted-slate transition-colors"
          >
            Live Control →
          </Link>
        </div>

      </section>

      <section
        id="how-it-works"
        className="w-full max-w-6xl flex flex-col items-center gap-2 py-16"
      >
        <p className="font-mono text-[48px] tracking-[0.28px] uppercase text-coral">
          The Hardware
        </p>

        <div className="w-full rounded-[22px] overflow-hidden -mt-12">
          <img
            src="/assembly.png"
            alt="RWIP CAD assembly — pendulum arm, reaction wheel, and motor base"
            className="w-full h-auto mix-blend-multiply"
          />
        </div>

        <p className="font-body text-[18px] leading-[1.4] text-ink max-w-3xl text-center">
          A brushed DC motor, custom  flywheel, rotary encoder, motor driver, and a microcontroller. Each component selected and integrated to make balance possible.
        </p>
      </section>
    </main>
  )
}
